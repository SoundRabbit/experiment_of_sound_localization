#define _USE_MATH_DEFINES

#include <utility>
#include <cmath>

# include <Siv3D.hpp> // OpenSiv3D v0.3.1

enum class Ear { Right, Left };

constexpr auto speedOfSound = 34029;	// cm/s

auto CalcSound(
	const Vec3& origin,	//頭の中心
	const double r,	//頭の半径
	const Vec3& source,	//音源の位置
	const Ear ear,	//どっちの耳か
	const Vec3& pinnacle,	//耳介ベクトル
	const Vec3& tragus,	//耳珠ベクトル
	const Vec3& externalAcousticOpening	//外耳孔ベクトル
) -> std::pair<double, double> {
	const auto earPos = ear == Ear::Left ? origin - Vec3(r, 0, 0) : origin + Vec3(r, 0, 0);
	const auto sourceVec = (source - earPos);

	if (sourceVec.dot(earPos) >= 0) {
		const auto normalizeSourceVec = sourceVec.normalized();
		const auto projectArea =
			std::max(0.0, normalizeSourceVec.dot(pinnacle)) +
			std::max(0.0, normalizeSourceVec.dot(tragus)) +
			std::max(0.0, normalizeSourceVec.dot(externalAcousticOpening));
		const auto distance2 = sourceVec.lengthSq();
		const auto distance = sourceVec.length();
		return { projectArea / distance2, distance / speedOfSound };
	}
	else {
		const auto normalizeSourceVec = Vec3(0, sourceVec.y, sourceVec.z).normalized();
		const auto projectArea =
			std::max(0.0, normalizeSourceVec.dot(pinnacle)) +
			std::max(0.0, normalizeSourceVec.dot(tragus)) +
			std::max(0.0, normalizeSourceVec.dot(externalAcousticOpening));
		const auto dist = source.length();	//反対側の耳への距離
		const auto distance = sqrt(dist*dist - r * r) + r * (acos(source.dot(earPos) / (r*dist)) - asin(r / dist));
		const auto distance2 = distance * distance;
		return { projectArea / distance2, distance / speedOfSound };
	}
}

auto CreateWave(
	const Vec3& origin,	//頭の中心
	const Vec3& source,	//音源の位置
	const double r = 10,	//頭の半径
	const size_t sampleNum = 4410,	//生成するサンプル数
	const size_t samplingRate = 44100	//サンプリングレート
) -> Wave {
	auto wave = Wave(sampleNum, s3d::Arg::samplingRate(samplingRate));

	constexpr auto earRotationYaw = M_PI / 6;	//耳とその位置の接面との傾き（Z軸回転）
	constexpr auto earRotationPitch = M_PI / 6;	//耳とその位置の接面との傾き（X軸回転）
	constexpr auto pinnacleAreaSize = 18.0;	//耳介の面積
	constexpr auto tragusAreaSize = 2;	//耳珠の面積
	constexpr auto eaoAreaSize = 1;	//外耳孔の面積

	assert(cos(earRotationYaw) > 0);

	const auto pinnacleL = Vec3(
		pinnacleAreaSize * cos(earRotationYaw) * (-1.0),
		pinnacleAreaSize * sin(earRotationYaw) * cos(earRotationPitch),
		pinnacleAreaSize * sin(earRotationYaw) * sin(earRotationPitch)
	);
	const auto tragusL = -pinnacleL * tragusAreaSize / pinnacleAreaSize;
	const auto eaoL = Vec3(
		eaoAreaSize * sin(earRotationYaw) * (-1.0),
		eaoAreaSize * cos(earRotationYaw) * cos(earRotationPitch) * (-1.0),
		eaoAreaSize * cos(earRotationYaw) * sin(earRotationPitch) * (-1.0)
	);

	const auto pinnacleR = Vec3(
		pinnacleAreaSize * cos(earRotationYaw),
		pinnacleAreaSize * sin(earRotationYaw) * cos(earRotationPitch),
		pinnacleAreaSize * sin(earRotationYaw) * sin(earRotationPitch)
	);
	const auto tragusR = -pinnacleL * tragusAreaSize / pinnacleAreaSize;
	const auto eaoR = Vec3(
		eaoAreaSize * sin(earRotationYaw),
		eaoAreaSize * cos(earRotationYaw) * cos(earRotationPitch) * (-1.0),
		eaoAreaSize * cos(earRotationYaw) * sin(earRotationPitch) * (-1.0)
	);

	const static auto attenuationLeft_Base = CalcSound(origin, r, Vec3(0.0, 100.0, 0.0), Ear::Left, pinnacleL, tragusL, eaoL).first;
	const static auto attenuationRight_Base = CalcSound(origin, r, Vec3(0.0, 100.0, 0.0), Ear::Right, pinnacleR, tragusR, eaoR).first;

	const auto[attenuationLeft, phaseLeft] = CalcSound(origin, r, source, Ear::Left, pinnacleL, tragusL, eaoL);
	const auto[attenuationRight, phaseRight] = CalcSound(origin, r, source, Ear::Right, pinnacleR, tragusR, eaoR);

	const auto volumeLeft = attenuationLeft / attenuationLeft_Base; // std::min(1.0, attenuationLeft / attenuationRight);
	const auto volumeRight = attenuationRight / attenuationRight_Base; // std::min(1.0, attenuationRight / attenuationLeft);

	constexpr auto freq = 440.0;

	for (int i = 0; i < wave.size(); i++) {
		wave[i] = WaveSample(
			sin((i*freq / wave.samplingRate() + phaseLeft)*M_PI * 2) * volumeLeft,
			sin((i*freq / wave.samplingRate() + phaseRight)*M_PI * 2) * volumeRight
		);
	}

	return std::move(wave);
}

void Main() {
	constexpr auto Blue = Color(89, 178, 255);
	constexpr auto CharacterColor = Color(84);
	constexpr auto unit = 100;

	Graphics::SetBackground(ColorF(0.8, 0.9, 1.0));

	const auto font = Font(40);

	Window::Resize(1920, 1080);

	const auto origin = Window::Center();

	const auto plane = Rect(origin - Point(300, 300), Size(600, 600));

	auto soundSourceLocation = Vec3(0, 0, 0);
	auto audio = Audio();
	auto audioBuffer = Audio();

	auto rotateVerticalCounter = 0;
	auto rotateVartical = false;

	auto rotate = false;
	constexpr auto angularVelocity = M_PI / 300.0; // rad/frame

	while (System::Update()) {
		if (plane.leftPressed()) {
			const auto relativeCursorPos = Cursor::Pos() - origin;
			soundSourceLocation.x = relativeCursorPos.x;
			soundSourceLocation.y = -relativeCursorPos.y;
		}

		SimpleGUI::VerticalSliderAt(
			soundSourceLocation.z,
			-3 * unit, 3 * unit,
			origin + Vec2(plane.size.x / 2 + 40, 0),
			plane.size.y);

		if (SimpleGUI::ButtonAt(
			U"リセット",
			origin + Vec2(plane.size.x / 2 + 40, plane.size.y / 2 + 40)
		)) {
			soundSourceLocation.z = 0;
		}

		if (SimpleGUI::ButtonAt(
			U"上下",
			origin + Vec2(plane.size.x / 2 + 40, plane.size.y / 2 + 80)
		)) {
			rotateVerticalCounter = 0;
			rotateVartical = !rotateVartical;
		}

		if (rotateVartical) {
			soundSourceLocation.z = sin(rotateVerticalCounter * angularVelocity) * 3 * unit;
			rotateVerticalCounter++;
		}

		if (SimpleGUI::ButtonAt(
			U"回転",
			origin + Vec2(-plane.size.x / 2 - 80, 0)
		)) {
			rotate = !rotate;
		}

		if (rotate) {
			soundSourceLocation.x = soundSourceLocation.x * cos(angularVelocity) - soundSourceLocation.y * sin(angularVelocity);
			soundSourceLocation.y = soundSourceLocation.x * sin(angularVelocity) + soundSourceLocation.y * cos(angularVelocity);
		}

		font(U"(", soundSourceLocation.x / unit, U" m , ", soundSourceLocation.y / unit, U" m , ", soundSourceLocation.z / unit, U" m)").drawAt(
			origin + Vec2(0, -plane.size.y / 2 - 40),
			CharacterColor
		);

		Graphics2D::SetScissorRect(plane);
		RasterizerState planeRasterizer = RasterizerState::Default2D;
		planeRasterizer.scissorEnable = true;
		auto render = RenderStateBlock2D(planeRasterizer);

		Window::ClientRect().draw(Palette::White);
		Circle(origin + Vec2(soundSourceLocation.x, -soundSourceLocation.y), 10).draw(Blue);
		Circle(origin, 10).draw(CharacterColor);

		if (!audio.isPlaying()) {
			std::swap(audio, audioBuffer);
			audio.play();
			audioBuffer = Audio(CreateWave(Vec3(0, 0, 0), soundSourceLocation));
		}
	}
}
