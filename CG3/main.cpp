#ifdef WIN32
#define _CRT_SECURE_NO_WARNINGS
#include <Windows.h>
#include <CommCtrl.h>
#endif

#include <iostream>
#include <sstream>
#include <fstream>
#include <future>
#include "glm/geometric.hpp"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include "renderer.h"

static Triangle make_triangle(Vec3 v0, Vec3 v1, Vec3 v2, Material *material) {
	Triangle t{ { v0, v1, v2 }, glm::normalize(glm::cross(v1 - v0, v2 - v0)), material };
	return t;
}

static Triangle make_triangle(Vec3 v0, glm::vec2 t0, Vec3 v1, glm::vec2 t1, Vec3 v2, glm::vec2 t2, Material *material) {
	Triangle t{ { v0, v1, v2 }, glm::normalize(glm::cross(v1 - v0, v2 - v0)), material };
	t.texCoord[0] = t0;
	t.texCoord[1] = t1;
	t.texCoord[2] = t2;
	return t;
}

static void readModel(Scene &scene, std::string path, float scaleFactor, Material *material) {
	std::ifstream f(path);
	std::string line;
	std::vector<Vec3> vs;

	while (std::getline(f, line)) {
		std::stringstream ss(line);
		std::string type;
		ss >> type;
		if (type == "v") {
			Vec3 v;
			ss >> v.x >> v.y >> v.z;
			vs.push_back(v * scaleFactor);
		}
		else if (type == "f") {
			std::string v_str;
			std::vector<int> f;
			while (ss >> v_str) {
				int v = atoi(v_str.c_str());
				if (v < 0)
					v += vs.size();
				else
					v--;
				f.push_back(v);
			}
			for (int i = 0; i < f.size() - 2; i++) {
				scene.triangles.push_back(make_triangle(vs[f[i]], vs[f[i + 1]], vs[f[i + 2]], material));
			}
		}
	}

	std::cout << "vertex: " << vs.size() << std::endl;
}
Material copper = { { 0.329412, 0.223529, 0.027451 },
{ 0.780392, 0.568627, 0.113725 },
{ 0.992157, 0.941176, 0.807843 },
27.8974,
0.2 };
Material chrome = { { 0.25, 0.25, 0.25 },
{ 0.4, 0.4, 0.4 },
{ 0.774597, 0.774597, 0.774597 },
76.8,
0.3 };
Material glass = { { 0.25, 0.25, 0.25 },
{ 0.4, 0.4, 0.4 },
{ 0.774597, 0.774597, 0.774597 },
76.8,
0.0 };
Material obsidian = {
		{ 0.05375, 0.05, 0.06625 },
		{ 0.18275, 0.17, 0.22525 },
		{ 0.332741, 0.328634, 0.346435 },
		38.4,
		0.0
};
static Color checkerTexture(glm::vec2 texCoord) {
	if (((int)(texCoord.x * 20) % 2 == 0) ^ ((int)(texCoord.y * 20) % 2 == 0))
		return Color(0, 0, 0);
	else
		return Color(1, 1, 1);
}

Material checker = {
		{ 0, 0, 0 },
		{ 1, 1, 1 },
		{ 0.332741, 0.328634, 0.346435 },
		38.4,
		0.0
};

Material wall1 = {
		{ 0, 0, 0 },
		{ 1, 1, 1 },
		{ 0.332741, 0.328634, 0.346435 },
		38.4,
		0.0
};
Material wall2 = {
		{ 0, 0, 0 },
		{ 1, 1, 1 },
		{ 0.332741, 0.328634, 0.346435 },
		38.4,
		0.0
};
Material wall3 = {
		{ 0, 0.5, 1 },
		{ 1, 1, 1 },
		{ 0.332741, 0.328634, 0.346435 },
		38.4,
		0.0
};

static void addPlane(Scene &scene, Vec3 lefttop, glm::vec2 uv0, Vec3 leftbottom, glm::vec2 uv1, Vec3 rightbottom, glm::vec2 uv2, Vec3 righttop, glm::vec2 uv3, Material *mat) {
	scene.triangles.push_back(make_triangle(lefttop, uv0, leftbottom, uv1, righttop, uv3, mat));
	scene.triangles.push_back(make_triangle(righttop, uv3, leftbottom, uv1, rightbottom, uv2, mat));
}

static void setupScene(Scene &scene) {
	checker.texFunc = checkerTexture;
	wall1.texFunc = [](glm::vec2 texCoord) { return Color(1, texCoord.y, 0); };
	wall2.texFunc = [](glm::vec2 texCoord) { return Color(0, texCoord.y, 1); };
	wall3.texFunc = [](glm::vec2 texCoord) { return Color(texCoord.y, 0, 1); };
	glass.refract = true;
	glass.refraction = 1.3f;
	glass.refractionFactor = 1.0f;
	//scene.spheres.push_back({ { 0.0, 0.2, 0.5 }, 0.02, &glass });
	//    scene.spheres.push_back({{-0.45, 0.1, -0.25}, 0.05, &copper});
	scene.spheres.push_back({ { 0.25, 0.1, 0.0 }, 0.1, &chrome });
	//    scene.spheres.push_back({{-0.2, 0.1, 0.0}, 0.05, &chrome});

	float w = 1.0, front = 2.0, back = -2.0, h = 2.0, y = -0.01f;
	addPlane(scene, { -w, y, back }, { 0, 0 }, { -w, y, front }, { 0, 1 }, { w, y, front }, { 1, 1 }, { w, y, back }, { 1, 0 }, &checker); // floor
	addPlane(scene, { -w, h, back }, { 0, 0 }, { -w, y, back }, { 0, 1 }, { w, y, back }, { 1, 1 }, { w, h, back }, { 1, 0 }, &wall2); // center
	addPlane(scene, { -w, h, front }, { 0, 0 }, { -w, y, front }, { 0, 1 }, { -w, y, back }, { 1, 1 }, { -w, h, back }, { 1, 0 }, &wall2); // left
	addPlane(scene, { w, h, back }, { 0, 0 }, { w, y, back }, { 0, 1 }, { w, y, front }, { 1, 1 }, { w, h, front }, { 1, 0 }, &wall2); // right
	readModel(scene, "2009210107_3.obj", 1.0, &glass);

	scene.camera.zNear = 0.01;
	scene.camera.zFar = 10.0;
	scene.camera.fovy = 60;
	scene.bgColor = { 0.0, 0.0, 0.0 };
	scene.lights.push_back({ LT_POINT, { 0.0, 0.5, 0.0 }, 1.0, { 1.0, 1.0, 1.0 } });
	//    scene.lights.push_back({LT_POINT, {0.5, 0.5, 0.5}, 0.5, {1.0, 0.0, 0.0}});
	//    scene.lights.push_back({LT_DIRECTIONAL, glm::normalize(Vec3({0.5f, -0.5f, 1.0f})), 1.0, {0.0, 1.0, 1.0}});
	//scene.lights.push_back({LT_DIRECTIONAL, glm::normalize(Vec3({0.5f, -0.5f, -1.0f})), 1.0, {1.0, 0.0, 1.0}});
	//scene.lights.push_back({ LT_DIRECTIONAL, glm::normalize(Vec3({ -0.5f, -0.5f, 0.0f })), 1.0, { 1.0, 1.0, 1.0 } });
	//scene.lights.push_back({LT_DIRECTIONAL, glm::normalize(Vec3({ -0.5f, -0.5f, -1.0f })), 2.0, { 1.0, 1.0, 1.0 } });
	scene.lights.push_back({ LT_SPOT, { -0.05, 0.2, 1.0 }, 3.0, { 0.0, 0.0, 1.0 }, cos(3 * 3.14159265358979323846f / 180.0f), glm::normalize(Vec3({ 0.1, 0.0, -1.0 })) });
}

#ifndef IDC_STATIC
#define IDC_STATIC (-1)
#endif

#define IDD_DIALOG1                             100
#define ID_STATUS                               40001
#define ID_CAMPOSY                              40002
#define ID_CAMPOSZ                              40003
#define ID_CAMUPX                               40004
#define ID_CAMATX                               40005
#define ID_CAMATY                               40006
#define ID_CAMATZ                               40007
#define ID_CAMPOSX                              40008
#define ID_CAMUPY                               40009
#define ID_CAMUPZ                               40010
#define ID_NRAYBOUNCE                           40011
#define ID_RESH                                 40012
#define ID_RESW                                 40013
#define ID_OCTREE                               40014
#define ID_PREVIEW                              40015

HWND ctrlWnd, renderWnd;
Scene scene;
unsigned int *pixels;
RenderParams params;
BITMAPINFO bitmapInfo;
time_t renderStartTime;

static void onTimer(HWND hwnd, UINT uMsg, UINT_PTR idEvent, DWORD dwTime) {
	InvalidateRect(renderWnd, NULL, FALSE);
}

static void renderWrapper() {
	render(scene, pixels, params);
	PostMessage(ctrlWnd, WM_USER + 1, 0, 0);
}

static float GetDlgItemFloat(int dlgItem) {
	WCHAR buf[20];
	UINT len = GetDlgItemText(ctrlWnd, dlgItem, buf, 20);
	std::wstringstream ss;
	ss.write(buf, len);
	float result;
	ss >> result;
	return result;
}

static void setupRenderParams() {
	int w = GetDlgItemInt(ctrlWnd, ID_RESW, NULL, FALSE);
	int h = GetDlgItemInt(ctrlWnd, ID_RESH, NULL, FALSE);
	delete pixels;
	pixels = new unsigned int[w * h];
	bitmapInfo.bmiHeader = { sizeof(BITMAPINFOHEADER), w, h, 1, 32, BI_RGB };
	scene.camera.aspect = (float)w / h;
	scene.camera.position = { GetDlgItemFloat(ID_CAMPOSX), GetDlgItemFloat(ID_CAMPOSY), GetDlgItemFloat(ID_CAMPOSZ) };
	scene.camera.at = { GetDlgItemFloat(ID_CAMATX), GetDlgItemFloat(ID_CAMATY), GetDlgItemFloat(ID_CAMATZ) };
	scene.camera.up = { GetDlgItemFloat(ID_CAMUPX), GetDlgItemFloat(ID_CAMUPY), GetDlgItemFloat(ID_CAMUPZ) };
	params.width = w;
	params.height = h;
	params.enableOctree = IsDlgButtonChecked(ctrlWnd, ID_OCTREE);
	params.depthLimit = GetDlgItemInt(ctrlWnd, ID_NRAYBOUNCE, NULL, FALSE);
	params.threads = 4;
	destroyOctree(scene);
	if (params.enableOctree)
		buildOctree(scene);
}

static void renderPreview() {
	setupRenderParams();
	RECT rect;
	rect.left = 0;
	rect.right = params.width;
	rect.top = 0;
	rect.bottom = params.height;
	AdjustWindowRectEx(&rect, WS_CAPTION | WS_SYSMENU, false, WS_EX_WINDOWEDGE | WS_EX_DLGMODALFRAME);
	SetWindowPos(renderWnd, 0, 0, 0, rect.right - rect.left, rect.bottom - rect.top, SWP_NOMOVE | SWP_SHOWWINDOW);
	SetDlgItemText(ctrlWnd, ID_STATUS, TEXT("Rendering..."));
	EnableWindow(GetDlgItem(ctrlWnd, ID_PREVIEW), FALSE);
	renderStartTime = time(NULL);
	std::async(renderWrapper);
	SetTimer(ctrlWnd, 1, 1000, (TIMERPROC)onTimer);
}

//static void renderToFile() {
//	setupRenderParams();
//	unsigned char *bytedata = new unsigned char[params.width * params.height * 3]; // RGB
//	int i = 0;
//	for (int y = h - 1; y >= 0; y--) {
//	    for (int x = 0; x < w; x++) {
//	        unsigned int c = pixels[y * w + x];
//	        bytedata[i++] = (c & 0xFF0000) >> 16; // r
//	        bytedata[i++] = (c & 0x00FF00) >> 8; // g
//	        bytedata[i++] = (c & 0x0000FF); // b
//	    }
//	}
//	stbi_write_png("out.png", w, h, 3, bytedata, 0);
//}

LRESULT CALLBACK WndProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
	switch (uMsg)
	{
	case WM_QUIT:
	case WM_DESTROY:
	case WM_CLOSE:
		PostQuitMessage(0);
		break;
	case WM_COMMAND:
		switch (LOWORD(wParam)) {
		case ID_PREVIEW:
			renderPreview();
			return 0;
		}
		break;
	case WM_USER + 1: {
		// render finished
		KillTimer(ctrlWnd, 1);
		int renderTime = time(NULL) - renderStartTime;
		std::wstringstream ss;
		ss << "Rendered (" << renderTime << " seconds)";
		SetDlgItemText(ctrlWnd, ID_STATUS, ss.str().c_str());
		EnableWindow(GetDlgItem(ctrlWnd, ID_PREVIEW), TRUE);
		InvalidateRect(renderWnd, NULL, FALSE);
		break;
	}
	default:
		return DefWindowProc(hWnd, uMsg, wParam, lParam);
	}
	return 0;
}

LRESULT CALLBACK WndProc2(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
	switch (uMsg)
	{
	case WM_CLOSE:
		// prevent WM_DESTROY
		ShowWindow(hWnd, SW_HIDE);
		return 0;
	case WM_PAINT: {
		PAINTSTRUCT ps;
		HDC hdc = BeginPaint(hWnd, &ps);
		SetDIBitsToDevice(hdc, 0, 0, bitmapInfo.bmiHeader.biWidth, bitmapInfo.bmiHeader.biHeight,
			0, 0, 0, bitmapInfo.bmiHeader.biHeight, &pixels[0], &bitmapInfo, DIB_RGB_COLORS);
		EndPaint(hWnd, &ps);
		break;
	}
	default:
		return DefWindowProc(hWnd, uMsg, wParam, lParam);
	}
	return 0;
}

int CALLBACK WinMain(
	HINSTANCE hInstance,
	HINSTANCE hPrevInstance,
	LPSTR     lpCmdLine,
	int       nCmdShow) {
	HINSTANCE hInst = GetModuleHandle(0);
	WNDCLASSEX wcex;
	ZeroMemory(&wcex, sizeof wcex);
	wcex.cbSize = sizeof wcex;
	wcex.hbrBackground = (HBRUSH)(COLOR_3DFACE + 1);
	wcex.lpszMenuName = 0;

	wcex.style = CS_HREDRAW | CS_VREDRAW;
	wcex.lpfnWndProc = WndProc;
	wcex.hInstance = hInst;
	wcex.hIcon = LoadIcon(0, (LPCTSTR)IDI_APPLICATION);
	wcex.hCursor = LoadCursor(NULL, IDC_ARROW);
	wcex.lpszClassName = TEXT("WndClass0");
	RegisterClassEx(&wcex);

	WNDCLASSEX wcex2;
	ZeroMemory(&wcex2, sizeof wcex2);
	wcex2.cbSize = sizeof wcex2;
	wcex2.hbrBackground = (HBRUSH)(COLOR_3DFACE + 1);
	wcex2.lpszMenuName = 0;

	wcex2.style = CS_HREDRAW | CS_VREDRAW;
	wcex2.lpfnWndProc = WndProc2;
	wcex2.hInstance = hInst;
	wcex2.hIcon = LoadIcon(0, (LPCTSTR)IDI_APPLICATION);
	wcex2.hCursor = LoadCursor(NULL, IDC_ARROW);
	wcex2.lpszClassName = TEXT("WndClass1");
	RegisterClassEx(&wcex2);

	HWND hwnd = CreateWindowEx(WS_EX_WINDOWEDGE | WS_EX_DLGMODALFRAME, TEXT("WndClass0"), TEXT("2009210107_Term"), WS_CAPTION | WS_VISIBLE | WS_SYSMENU, CW_USEDEFAULT, CW_USEDEFAULT, 300, 380, 0, 0, hInst, 0);
	HWND hCtrl0_0 = CreateWindowEx(0, WC_STATIC, TEXT("Status: Ready"), WS_VISIBLE | WS_CHILD | WS_GROUP | SS_LEFT, 8, 312, 263, 20, hwnd, (HMENU)ID_STATUS, hInst, 0);
	HWND hCtrl0_1 = CreateWindowEx(0, WC_STATIC, TEXT("Ray bounce"), WS_VISIBLE | WS_CHILD | WS_GROUP | SS_LEFT, 8, 216, 100, 15, hwnd, (HMENU)0, hInst, 0);
	HWND hCtrl0_2 = CreateWindowEx(0, WC_EDIT, TEXT("1280"), WS_VISIBLE | WS_CHILD | WS_TABSTOP | WS_BORDER | ES_AUTOHSCROLL | ES_NUMBER, 98, 187, 75, 21, hwnd, (HMENU)ID_RESW, hInst, 0);
	HWND hCtrl0_3 = CreateWindowEx(0, WC_STATIC, TEXT("x"), WS_VISIBLE | WS_CHILD | WS_GROUP | SS_LEFT, 182, 190, 6, 15, hwnd, (HMENU)0, hInst, 0);
	HWND hCtrl0_4 = CreateWindowEx(0, WC_STATIC, TEXT("Resolution"), WS_VISIBLE | WS_CHILD | WS_GROUP | SS_LEFT, 8, 187, 70, 15, hwnd, (HMENU)0, hInst, 0);
	HWND hCtrl0_7 = CreateWindowEx(0, WC_STATIC, TEXT("Camera Position"), WS_VISIBLE | WS_CHILD | WS_GROUP | SS_LEFT, 8, 16, 78, 15, hwnd, (HMENU)0, hInst, 0);
	HWND hCtrl0_8 = CreateWindowEx(0, WC_STATIC, TEXT("X:"), WS_VISIBLE | WS_CHILD | WS_GROUP | SS_LEFT, 8, 41, 12, 15, hwnd, (HMENU)0, hInst, 0);
	HWND hCtrl0_9 = CreateWindowEx(0, WC_BUTTON, TEXT("Preview"), WS_VISIBLE | WS_CHILD | WS_TABSTOP | 0x00000001, 8, 280, 128, 23, hwnd, (HMENU)ID_PREVIEW, hInst, 0);
	HWND hCtrl0_10 = CreateWindowEx(0, WC_BUTTON, TEXT("Use Octree"), WS_VISIBLE | WS_CHILD | WS_TABSTOP | 0x00000003, 8, 247, 100, 13, hwnd, (HMENU)ID_OCTREE, hInst, 0);
	HWND hCtrl0_11 = CreateWindowEx(0, WC_STATIC, TEXT("Y:"), WS_VISIBLE | WS_CHILD | WS_GROUP | SS_LEFT, 105, 41, 12, 15, hwnd, (HMENU)0, hInst, 0);
	HWND hCtrl0_12 = CreateWindowEx(0, WC_STATIC, TEXT("Z:"), WS_VISIBLE | WS_CHILD | WS_GROUP | SS_LEFT, 188, 41, 12, 15, hwnd, (HMENU)0, hInst, 0);
	HWND hCtrl0_5 = CreateWindowEx(0, WC_EDIT, TEXT("0"), WS_VISIBLE | WS_CHILD | WS_TABSTOP | WS_BORDER | ES_AUTOHSCROLL, 23, 41, 53, 20, hwnd, (HMENU)ID_CAMPOSX, hInst, 0);
	HWND hCtrl0_13 = CreateWindowEx(0, WC_EDIT, TEXT("0.2"), WS_VISIBLE | WS_CHILD | WS_TABSTOP | WS_BORDER | ES_AUTOHSCROLL, 120, 41, 53, 20, hwnd, (HMENU)ID_CAMPOSY, hInst, 0);
	HWND hCtrl0_14 = CreateWindowEx(0, WC_EDIT, TEXT("2"), WS_VISIBLE | WS_CHILD | WS_TABSTOP | WS_BORDER | ES_AUTOHSCROLL, 203, 41, 53, 20, hwnd, (HMENU)ID_CAMPOSZ, hInst, 0);
	HWND hCtrl0_15 = CreateWindowEx(0, WC_EDIT, TEXT("720"), WS_VISIBLE | WS_CHILD | WS_TABSTOP | WS_BORDER | ES_AUTOHSCROLL | ES_NUMBER, 195, 187, 75, 21, hwnd, (HMENU)ID_RESH, hInst, 0);
	HWND hCtrl0_16 = CreateWindowEx(0, WC_EDIT, TEXT("2"), WS_VISIBLE | WS_CHILD | WS_TABSTOP | WS_BORDER | ES_AUTOHSCROLL | ES_NUMBER, 98, 215, 75, 21, hwnd, (HMENU)ID_NRAYBOUNCE, hInst, 0);
	HWND hCtrl0_17 = CreateWindowEx(0, WC_STATIC, TEXT("Camera Look At"), WS_VISIBLE | WS_CHILD | WS_GROUP | SS_LEFT, 8, 73, 78, 15, hwnd, (HMENU)0, hInst, 0);
	HWND hCtrl0_19 = CreateWindowEx(0, WC_STATIC, TEXT("X:"), WS_VISIBLE | WS_CHILD | WS_GROUP | SS_LEFT, 8, 98, 12, 15, hwnd, (HMENU)0, hInst, 0);
	HWND hCtrl0_20 = CreateWindowEx(0, WC_STATIC, TEXT("Y:"), WS_VISIBLE | WS_CHILD | WS_GROUP | SS_LEFT, 105, 98, 12, 15, hwnd, (HMENU)0, hInst, 0);
	HWND hCtrl0_21 = CreateWindowEx(0, WC_STATIC, TEXT("Z:"), WS_VISIBLE | WS_CHILD | WS_GROUP | SS_LEFT, 188, 98, 12, 15, hwnd, (HMENU)0, hInst, 0);
	HWND hCtrl0_18 = CreateWindowEx(0, WC_EDIT, TEXT("0.0"), WS_VISIBLE | WS_CHILD | WS_TABSTOP | WS_BORDER | ES_AUTOHSCROLL, 23, 98, 53, 20, hwnd, (HMENU)ID_CAMATX, hInst, 0);
	HWND hCtrl0_22 = CreateWindowEx(0, WC_EDIT, TEXT("0.2"), WS_VISIBLE | WS_CHILD | WS_TABSTOP | WS_BORDER | ES_AUTOHSCROLL, 120, 98, 53, 20, hwnd, (HMENU)ID_CAMATY, hInst, 0);
	HWND hCtrl0_23 = CreateWindowEx(0, WC_EDIT, TEXT("0.1"), WS_VISIBLE | WS_CHILD | WS_TABSTOP | WS_BORDER | ES_AUTOHSCROLL, 203, 98, 53, 20, hwnd, (HMENU)ID_CAMATZ, hInst, 0);
	HWND hCtrl0_24 = CreateWindowEx(0, WC_STATIC, TEXT("Camera Up Vector"), WS_VISIBLE | WS_CHILD | WS_GROUP | SS_LEFT, 8, 130, 89, 15, hwnd, (HMENU)0, hInst, 0);
	HWND hCtrl0_25 = CreateWindowEx(0, WC_EDIT, TEXT("0.0"), WS_VISIBLE | WS_CHILD | WS_TABSTOP | WS_BORDER | ES_AUTOHSCROLL, 23, 154, 53, 20, hwnd, (HMENU)ID_CAMUPX, hInst, 0);
	HWND hCtrl0_26 = CreateWindowEx(0, WC_STATIC, TEXT("X:"), WS_VISIBLE | WS_CHILD | WS_GROUP | SS_LEFT, 8, 154, 12, 15, hwnd, (HMENU)0, hInst, 0);
	HWND hCtrl0_27 = CreateWindowEx(0, WC_STATIC, TEXT("Y:"), WS_VISIBLE | WS_CHILD | WS_GROUP | SS_LEFT, 105, 154, 12, 15, hwnd, (HMENU)0, hInst, 0);
	HWND hCtrl0_28 = CreateWindowEx(0, WC_STATIC, TEXT("Z:"), WS_VISIBLE | WS_CHILD | WS_GROUP | SS_LEFT, 188, 154, 12, 15, hwnd, (HMENU)0, hInst, 0);
	HWND hCtrl0_29 = CreateWindowEx(0, WC_EDIT, TEXT("1.0"), WS_VISIBLE | WS_CHILD | WS_TABSTOP | WS_BORDER | ES_AUTOHSCROLL, 120, 154, 53, 20, hwnd, (HMENU)ID_CAMUPY, hInst, 0);
	HWND hCtrl0_30 = CreateWindowEx(0, WC_EDIT, TEXT("0.0"), WS_VISIBLE | WS_CHILD | WS_TABSTOP | WS_BORDER | ES_AUTOHSCROLL, 203, 154, 53, 20, hwnd, (HMENU)ID_CAMUPZ, hInst, 0);
	CheckDlgButton(hwnd, ID_OCTREE, BST_CHECKED);

	ctrlWnd = hwnd;
	renderWnd = CreateWindowEx(WS_EX_WINDOWEDGE | WS_EX_DLGMODALFRAME, TEXT("WndClass1"), TEXT("Preview"), WS_CAPTION | WS_SYSMENU, CW_USEDEFAULT, CW_USEDEFAULT, 0, 0, 0, 0, hInst, 0);
	setupScene(scene);

	MSG msg;
	while (GetMessage(&msg, NULL, 0, 0))
	{
		TranslateMessage(&msg);
		DispatchMessage(&msg);
	}

	return (int)msg.wParam;
}
