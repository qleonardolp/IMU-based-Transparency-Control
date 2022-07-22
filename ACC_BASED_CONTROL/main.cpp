//|///////////////////////////\_____///\////_____ ___  ___ \//|
//|Leonardo Felipe Lima Santos dos Santos/  | |  | . \/   \ \/|
//|github/bitbucket qleonardolp        //\ 	| |   \ \   |_|  \|
//|License: BSD (2022) ////\__________////\ \_'_/\_`_/__|   //|

#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl3.h"
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <GLES2/gl2.h>
#endif
#include <GLFW/glfw3.h> // Will drag system OpenGL headers

// [Win32] Our example includes a copy of glfw3.lib pre-compiled 
// with VS2010 to maximize ease of testing and compatibility with old VS compilers.
// To link with VS2010-era libraries, VS2015+ requires linking with legacy_stdio_definitions.lib, 
// which we do using this pragma. Your own project should not be affected, as you are likely to 
// link with a newer binary of GLFW that is adequate for your version of Visual Studio.
//#if defined(_MSC_VER) && (_MSC_VER >= 1900) && !defined(IMGUI_DISABLE_WIN32_FUNCTIONS)
//#pragma comment(lib, "legacy_stdio_definitions")
//#endif
// comentado pois baixei a glfw3.lib mais recente para VS2022!

#include "implot.h"

#include "SharedStructs.h" // ja inclui <stdio.h> / <thread> / <mutex> / <vector>
#if CAN_ENABLE
#include "XsensEpos.h"
#endif
#include "QpcLoopTimer.h" // ja inclui <windows.h>
#include <processthreadsapi.h>
#include <stdexcept>
#include <iostream>
#include <conio.h>

void opmode01(short* opt);
void opmode02(short* opt);
void opmode03(short* opt);
void opmode04(short* opt);
void opmode05(short* opt);
void opmode06(short* opt);
void opmode07(short* opt);
void opmode08(short* opt);
void readIMUs(ThrdStruct& data_struct);
void readFTSensor(ThrdStruct& data_struct);
void qASGD(ThrdStruct& data_struct);
void Controle(ThrdStruct& data_struct);
void Logging(ThrdStruct& data_struct);
void updateGains(ThrdStruct& data_struct);

// DEBUGGING DEFINES:
#define PRIORITY     0
#define EXEC_TIME   25

// Threads Sample Time:
#define IMU_SMPLTM  0.01333 // "@75 Hz", actually is defined by Xsens 'desiredUpdateRate'
#define ASGD_SMPLTM IMU_SMPLTM //
#define CTRL_SMPLTM 0.0010 //  @1000 Hz
#define LOG_SMPLTM  0.0050 //  @200  Hz 
#define GSCN_SMPLTM 4.0000 //  @ leitura de ganhos do arquivo a cada 4s 
#define FT_SMPLTM   0.0010 //  @1000 Hz, pode chegar a 7kHz ... 
// Threads Priority:
#define IMU_PRIORITY       -1 //
#define ASGD_PRIORITY      -1 //
#define CTRL_PRIORITY      -2 //
#define LOG_PRIORITY        0 //
#define DEFAULT_PRIORITY    0 // 

// Global Vars
bool imu_start(false);
bool asgd_start(false);
bool control_start(false);
bool logging_start(false);
bool ftsensor_start(false);
bool gscan_start(false);

int  execution_time = EXEC_TIME;

// utility structure for realtime plot
struct RollingBuffer {
	float Span;
	ImVector<ImVec2> Data;
	RollingBuffer() {
		Span = 10.0f;
		Data.reserve(2000);
	}
	void AddPoint(float x, float y) {
		float xmod = fmodf(x, Span);
		if (!Data.empty() && xmod < Data.back().x)
			Data.shrink(0);
		Data.push_back(ImVec2(xmod, y));
	}
};

static void glfw_error_callback(int error, const char* description)
{
	fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

int main(int, char**)
{
	using namespace std;
#if PRIORITY
	DWORD dwPriority, dwError;
	if (!SetPriorityClass(GetCurrentProcess(), HIGH_PRIORITY_CLASS)) {
		dwError = GetLastError();
		printf("Error %lu! \n", dwError);
	}
	else {
		dwPriority = GetPriorityClass(GetCurrentProcess());
		printf("Process priority class: %lu \n", dwPriority);
	}
#endif

#if ( CAN_ENABLE && false)
	cout << "INICIALIZANDO COMUNICACAO CANOpen COM AS EPOS" << endl;
	IniciaRedeCan();
#endif

	mutex comm_mtx;
	float imu_data[DTVC_SZ];
	float gains_data[DTVC_SZ];
	float logging_data[DTVCA_SZ];
	float states_data[DTVCB_SZ];
	float ati_data[DTVCF_SZ];
	//initialize data vectors (safety):
	for (int i = 0; i < DTVC_SZ; i++) {
		imu_data[i] = gains_data[i] = 0;
		if (i < 10) logging_data[i] = states_data[i] = 0;
		if (i < 6) ati_data[i] = 0;
	}

	static short imu_isready(false);
	static short asgd_isready(false);
	static short control_isready(false);
	static short logging_isready(false);
	static short ftsensor_isready(false);
	static short gscan_isready(false);

	static short imu_aborting(false);
	static short asgd_aborting(false);
	static short control_aborting(false);
	static short logging_aborting(false);
	static short ftsensor_aborting(false);
	static short gscan_aborting(false);

	short finished[10] = { 0 }; // finished flags
	bool th_running = false; // TODO: flag para evitar que na GUI uma nova execucao seja iniciada enquanto ha um rodando...

	ThrdStruct imu_struct, asgd_struct, ftsensor_struct;
	ThrdStruct control_struct, logging_struct, gscan_struct;

	// Function Structs definition:
	{ // Apenas para colapsar e facilitar leitura do código

	// IMU Struct:
		imu_struct.sampletime_ = IMU_SMPLTM;
		imu_struct.param00_ = IMU_PRIORITY;
		imu_struct.param0A_ = &imu_isready;
		imu_struct.param1A_ = &imu_aborting;
		imu_struct.param3F_ = finished + 0;
		*(imu_struct.datavec_) = imu_data;
		*(imu_struct.datavecB_) = states_data;
		imu_struct.mtx_ = &comm_mtx;

		// qASGD Struct:
		asgd_struct.sampletime_ = ASGD_SMPLTM;
		asgd_struct.param00_ = ASGD_PRIORITY;
		asgd_struct.param0B_ = &asgd_isready;
		asgd_struct.param1B_ = &asgd_aborting;
		asgd_struct.param0A_ = &imu_isready;
		asgd_struct.param1A_ = &imu_aborting;
		asgd_struct.param3F_ = finished + 1;
		*(asgd_struct.datavec_) = imu_data;     // from IMUs
		*(asgd_struct.datavecA_) = logging_data;// to logging
		*(asgd_struct.datavecB_) = states_data; // to control
		asgd_struct.mtx_ = &comm_mtx;

		// Control Struct
		control_struct.sampletime_ = CTRL_SMPLTM;
		control_struct.param00_ = CTRL_PRIORITY;
		control_struct.param0C_ = &control_isready;
		control_struct.param1C_ = &control_aborting;
		control_struct.param0A_ = &imu_isready;
		control_struct.param1A_ = &imu_aborting;
		control_struct.param0B_ = &asgd_isready;
		control_struct.param0D_ = &logging_isready;
		control_struct.param0E_ = &ftsensor_isready;
		control_struct.param1E_ = &ftsensor_aborting;
		control_struct.param0F_ = &gscan_isready;
		control_struct.param3F_ = finished + 2;
		*(control_struct.datavec_) = gains_data;
		*(control_struct.datavecA_) = logging_data;
		*(control_struct.datavecB_) = states_data;
		*(control_struct.datavecF_) = ati_data;
		control_struct.mtx_ = &comm_mtx;

		// Logging Struct
		logging_struct.sampletime_ = LOG_SMPLTM;
		logging_struct.param00_ = LOG_PRIORITY;
		logging_struct.param0D_ = &logging_isready;
		logging_struct.param1D_ = &logging_aborting;
		logging_struct.param0A_ = &imu_isready;
		logging_struct.param1A_ = &imu_aborting;
		logging_struct.param0B_ = &asgd_isready;
		logging_struct.param1B_ = &asgd_aborting;
		logging_struct.param0C_ = &control_isready;
		logging_struct.param1C_ = &control_aborting;
		logging_struct.param0E_ = &ftsensor_isready;
		logging_struct.param3F_ = finished + 3;
		*(logging_struct.datavec_) = gains_data;
		*(logging_struct.datavecA_) = logging_data;
		*(logging_struct.datavecB_) = states_data;
		*(logging_struct.datavecF_) = ati_data;
		logging_struct.mtx_ = &comm_mtx;

		// F/T Sensor Struct
		ftsensor_struct.sampletime_ = FT_SMPLTM;
		ftsensor_struct.param00_ = DEFAULT_PRIORITY;
		ftsensor_struct.param0E_ = &ftsensor_isready;
		ftsensor_struct.param1E_ = &ftsensor_aborting;
		ftsensor_struct.param0A_ = &imu_isready;
		ftsensor_struct.param1A_ = &imu_aborting;
		ftsensor_struct.param3F_ = finished + 4;
		*(ftsensor_struct.datavecB_) = states_data;
		*(ftsensor_struct.datavecF_) = ati_data;
		ftsensor_struct.mtx_ = &comm_mtx;

		// updateGains Struct
		gscan_struct.sampletime_ = GSCN_SMPLTM;
		gscan_struct.param00_ = DEFAULT_PRIORITY;
		gscan_struct.param0F_ = &gscan_isready;
		gscan_struct.param1F_ = &gscan_aborting;
		gscan_struct.param0D_ = &logging_isready;
		gscan_struct.param0C_ = &control_isready;
		gscan_struct.param3F_ = finished + 5;
		*(gscan_struct.datavec_) = gains_data;
		gscan_struct.mtx_ = &comm_mtx;
	}
	// Threads declaration
	thread thr_imus;
	thread thr_qasgd;
	thread thr_controle;
	thread thr_logging;
	thread thr_ftsensor;
	thread thr_gainscan;

	// Setup window
	glfwSetErrorCallback(glfw_error_callback);
	if (!glfwInit())
		return 1;

	// Decide GL+GLSL versions
#if defined(IMGUI_IMPL_OPENGL_ES2)
	// GL ES 2.0 + GLSL 100
	const char* glsl_version = "#version 100";
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
	glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
#elif defined(__APPLE__)
	// GL 3.2 + GLSL 150
	const char* glsl_version = "#version 150";
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // Required on Mac
#else
	// GL 3.0 + GLSL 130
	const char* glsl_version = "#version 130";
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
	//glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
	//glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // 3.0+ only
#endif

	// Create window with graphics context
	GLFWwindow* window = glfwCreateWindow(1820, 1080, "Project Hylonome | D.ImGui[GLFW/OpenGL3]+ImPlot", NULL, NULL);
	if (window == NULL) return 1;
	glfwSetWindowPos(window, 0, 38);
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1); // Enable vsync

	// Setup Dear ImGui context
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO(); (void)io;
	//io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
	//io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

	ImPlot::CreateContext();

	ImGui::LoadIniSettingsFromDisk("imgui.ini");
	// Setup Dear ImGui style
	ImGui::StyleColorsLight();
	ImGui::GetStyle().FrameRounding = 3;
	ImGui::GetStyle().FrameBorderSize = 1;
	ImGui::GetStyle().AntiAliasedLinesUseTex = false;
	ImGui::GetStyle().AntiAliasedFill = false;
	ImGui::GetStyleColorVec4(ImGuiCol_WindowBg);

	// Setup Platform/Renderer backends
	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init(glsl_version);

	// Our state
	bool show_demo_window = false;
	bool show_another_window = false;
	ImVec4 clear_color = ImVec4(0.6f, 0.6f, 0.6f, 1.00f);

	bool show_imu_window = false;
	bool show_ctrl_window = false;

	// Main loop
	while (!glfwWindowShouldClose(window))
	{
		// Poll and handle events (inputs, window resize, etc.)
		// You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
		// - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application.
		// - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application.
		// Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.
		glfwPollEvents();

		// Start the Dear ImGui frame
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();

		// 1. Show the big demo window (Most of the sample code is in ImGui::ShowDemoWindow()! You can browse its code to learn more about Dear ImGui!).
		if (show_demo_window)
			ImGui::ShowDemoWindow(&show_demo_window);

		// 2. Show a simple window that we create ourselves. We use a Begin/End pair to created a named window.
		{
			static float f = 0.0f;
			static int counter = 0;
			static short option = 0;

			ImGui::Begin("Hello, ExoTau!");                          // Create a window with string name...

			ImGui::Text(" //////////////////////////////////////////\\/////////\\/");
			ImGui::Text(" // INTERFACE DE CONTROLE EXO-TAU  /       /\\     ////\\");
			ImGui::Text(" // EESC-USP          [%.1f FPS]  / _____ ___  ___  //|", ImGui::GetIO().Framerate);
			ImGui::Text(" // RehabLab                     /  | |  | . \\/   \\  /|");
			ImGui::Text(" // *Copyright 2021-2026* \\//// //  | |   \\ \\   |_|  /|");
			ImGui::Text(" //\\///////////////////////\\// //// \\_'_/\\_`_/__|   ///");
			ImGui::Text(" ///\\///////////////////////\\ //////////////////\\/////\\");

			ImGui::Checkbox("Demo Window", &show_demo_window);      // Edit bools storing our window open/close state
			ImGui::Checkbox("Another Window", &show_another_window);

			//ImGui::SliderFloat("float", &f, -5.0f, 5.0f);           // Edit 1 float using a slider
			//ImGui::ColorEdit3("clear color", (float*)&clear_color); // Edit 3 floats representing a color

			if (ImGui::Button("Button")) counter++;                   // Buttons return true when clicked (most widgets return true when edited/activated)
			ImGui::SameLine();
			ImGui::Text("counter = %d", counter);

			if (ImGui::Button("Controle com IMUs + F/T"))
				opmode01(&option);

			ImGui::SameLine();

			if (ImGui::Button("Controle com IMUs"))
				opmode02(&option);

			if (ImGui::Button("Controle 'IMU3 bypass'"))
				opmode03(&option);

			ImGui::SameLine();

			if (ImGui::Button("Controle sem IMUs (MF SEA)"))
				opmode08(&option);

			if (ImGui::Button("Leitura IMUs"))
				opmode04(&option);

			ImGui::SameLine();

			if (ImGui::Button("Leitura F/T"))
				opmode05(&option);

			ImGui::SameLine();

			if (ImGui::Button("Leitura Parametros"))
				opmode06(&option);

			if (ImGui::Button("Reset Falhas CAN"))
			{
				opmode07(&option);
#if CAN_ENABLE
				ResetRedeCan();
				HabilitaEixo(2);
				DesabilitaEixo(2);
#endif
			}

			imu_struct.param39_ = option;
			asgd_struct.param39_ = option;
			control_struct.param39_ = option;
			logging_struct.param39_ = option;
			// Fill structs exectime:
			{
				imu_struct.exectime_ = execution_time;
				asgd_struct.exectime_ = execution_time;
				gscan_struct.exectime_ = execution_time;
				control_struct.exectime_ = execution_time;
				logging_struct.exectime_ = execution_time;
				ftsensor_struct.exectime_ = execution_time;
			}
			//ImGui::Text("Tempo de execução (s):");
			//ImGui::SameLine();
			if (ImGui::InputInt("segundos", &execution_time, 1, 20)) {
				imu_struct.exectime_ = execution_time;
				asgd_struct.exectime_ = execution_time;
				gscan_struct.exectime_ = execution_time;
				control_struct.exectime_ = execution_time;
				logging_struct.exectime_ = execution_time;
				ftsensor_struct.exectime_ = execution_time;
			}

			// Fill an array of contiguous float values to plot
			// Tip: If your float aren't contiguous but part of a structure, you can pass a pointer to your first float
			// and the sizeof() of your structure in the "stride" parameter.
			static bool animate = true;
			static float values[90] = {};
			static int values_offset = 0;
			static double refresh_time = 0.0;
			if (!animate || refresh_time == 0.0)
				refresh_time = ImGui::GetTime();
			while (refresh_time < ImGui::GetTime()) // Create data at fixed 60 Hz rate for the demo
			{
				static float phase = 0.0f;
				values[values_offset] = cosf(phase);
				values_offset = (values_offset + 1) % IM_ARRAYSIZE(values);
				phase += 0.05f * values_offset;
				refresh_time += 1.0f / 60.0f;
			}

			// Plots can display overlay texts
			// (in this example, we will display an average value)
			{
				float average = 0.0f;
				for (int n = 0; n < IM_ARRAYSIZE(values); n++)
					average += values[n] * values[n];
				average /= (float)IM_ARRAYSIZE(values);
				average = sqrtf(average);
				char overlay[32];
				sprintf(overlay, "rms %f", average);
				ImGui::PlotLines("Lines", values, IM_ARRAYSIZE(values), values_offset, overlay, -1.0f, 1.0f, ImVec2(0, 80.0f));
			}
			ImGui::Checkbox("Animate", &animate);

			ImGui::End();
		}

		// Escolher tipo de controle:
		if (control_start || show_ctrl_window) {
			show_ctrl_window = true;
			ImGui::Begin("Controlador", &show_ctrl_window);
			ImGui::Text(" Escolha uma controlador:");
			static short option = -1;
			if (ImGui::Button("Acc-based Transparency (PD)")) option = 1;
			if (ImGui::Button("Acc-based Transparency Z_0")) option = 2;
			if (ImGui::Button("Acc-based Transparency (LS)")) option = 3;
			if (ImGui::Button("Int-based Transparency (F/T)")) option = 4;
			if (ImGui::Button("Markovian Transparency (Mao)")) option = 5;
			if (ImGui::Button("Malha Fechada SEA (MF SEA)"))   option = 8;
			if (ImGui::Button("GyroscopeX 'IMU3 bypass'"))   option = 33;
			// to logging the control option
			control_struct.param01_ = option;
			logging_struct.param10_ = option;
			ftsensor_struct.param01_ = true; // F/T filtering on/off
			ImGui::End();
		}

		// Janela de graficos (realtime) IMUs:
		if (imu_start || show_imu_window)
		{
			show_imu_window = true;
			ImGui::Begin("Joint Angle Estimator", &show_imu_window);
			ImGui::BulletText("This example assumes 60 FPS. Higher FPS requires larger buffer size.");
			static RollingBuffer   dataKneePos, dataAnklePos;
			ImVec2 mouse = ImGui::GetMousePos();
			static float t = 0;
			t += ImGui::GetIO().DeltaTime;
			//dataKneePos.AddPoint(t, mouse.x * 0.0005f);
			//dataAnklePos.AddPoint(t, mouse.y * 0.0005f);

			static float history = 10.0f;
			ImGui::SliderFloat("History", &history, 1, 30, "%.1f s");
			dataKneePos.Span = history;
			dataAnklePos.Span = history;

			static ImPlotAxisFlags flags = ImPlotAxisFlags_NoTickLabels;

			if (ImPlot::BeginPlot("Joint Positions", ImVec2(-1, 150))) {
				
				unique_lock<mutex> _(comm_mtx);
				float right_knee_pos  = states_data[0];
				float right_ankle_pos = states_data[3];
				dataKneePos.AddPoint(t, right_knee_pos * 180.0f / 3.14159f);
				dataAnklePos.AddPoint(t, right_ankle_pos * 180.0f / 3.14159f);

				// IMU Abort close this window:
				//show_imu_window = !(*imu_struct.param1A_);

				ImPlot::SetupAxes(NULL, NULL, flags, flags);
				ImPlot::SetupAxisLimits(ImAxis_X1, 0, history, ImGuiCond_Always);
				ImPlot::SetupAxisLimits(ImAxis_Y1, -100, 100, ImGuiCond_Always);
				ImPlot::PlotLine("Knee", &dataKneePos.Data[0].x, &dataKneePos.Data[0].y, dataKneePos.Data.size(), 0, 2 * sizeof(float));
				ImPlot::PlotLine("Ankle", &dataAnklePos.Data[0].x, &dataAnklePos.Data[0].y, dataAnklePos.Data.size(), 0, 2 * sizeof(float));
				ImPlot::EndPlot();
			}
			ImGui::End();
		}

		// 3. Show another simple window.
		if (show_another_window)
		{
			// Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)
			ImGui::Begin("Another Window", &show_another_window);
			ImGui::Text("Hello from another window!");
			if (ImGui::Button("Close Me"))
				show_another_window = false;
			ImGui::End();
		}

		// TODO: alguma thread esta rodando sem a outra que eh dependente rodar...
		// Fire Threads:
		if (imu_start) {
			thr_imus = thread(readIMUs, imu_struct);
			imu_start = false;
		}
		if (asgd_start) {
			thr_qasgd = thread(qASGD, asgd_struct);
			asgd_start = false;
		}
		if (logging_start) {
			thr_logging = thread(Logging, logging_struct);
			logging_start = false;
		}
		if (control_start) {
			thr_controle = thread(Controle, control_struct);
			control_start = false;
		}
		if (ftsensor_start) {
			thr_ftsensor = thread(readFTSensor, ftsensor_struct);
			ftsensor_start = false;
		}
		if (gscan_start) {
			thr_gainscan = thread(updateGains, gscan_struct);
			gscan_start = false;
		}

		// Join Threads:
		{
			unique_lock<mutex> _(comm_mtx);
			if (*imu_struct.param3F_) {
				thr_imus.join();
				*imu_struct.param3F_ = 0; // restart
				cout << "-> readIMU jointed!" << endl;
			}
			if (*asgd_struct.param3F_) {
				thr_qasgd.join();
				*asgd_struct.param3F_ = 0;
				cout << "-> qASGD jointed!" << endl;
			}
			if (*logging_struct.param3F_) {
				thr_logging.join();
				*logging_struct.param3F_ = 0;
				cout << "-> Logging jointed!" << endl;
			}
			if (*control_struct.param3F_) {
				thr_controle.join();
				*control_struct.param3F_ = 0;
				cout << "-> Control jointed!" << endl;
			}
			if (*ftsensor_struct.param3F_) {
				thr_ftsensor.join();
				*ftsensor_struct.param3F_ = 0;
				cout << "-> FT Sensor jointed!" << endl;
			}
			if (*gscan_struct.param3F_) {
				thr_gainscan.join();
				*gscan_struct.param3F_ = 0;
				cout << "-> GainScan jointed!" << endl;
			}
		}

		// Rendering
		ImGui::Render();
		int display_w, display_h;
		glfwGetFramebufferSize(window, &display_w, &display_h);
		glViewport(0, 0, display_w, display_h);
		glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
		glClear(GL_COLOR_BUFFER_BIT);
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

		glfwSwapBuffers(window);
	}

#if CAN_ENABLE
	epos.StopPDOS(1);
#endif
	std::cout << " Successful exit." << endl;
	// ImGui cleanup:
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImPlot::DestroyContext();
	ImGui::DestroyContext();

	glfwDestroyWindow(window);
	glfwTerminate();
	return 0;
}

void opmode01(short* opt) {
	*opt = 1;
	// tudo roda:
	imu_start = asgd_start = true;
	gscan_start = logging_start = true;
	control_start = ftsensor_start = true;
}

void opmode02(short* opt) {
	*opt = 2;
	asgd_start = imu_start = true;
	gscan_start = true;
	logging_start = true;
	control_start = true;
	ftsensor_start = false;
}

void opmode03(short* opt) {
	*opt = IMUBYPASS;
	asgd_start = imu_start = true;
	gscan_start = true;
	logging_start = true;
	control_start = true;
	ftsensor_start = false;
}

void opmode04(short* opt) {
	*opt = READIMUS;
	asgd_start = imu_start = true;
	gscan_start = false;
	logging_start = true;
	control_start = false;
	ftsensor_start = false;
}

void opmode05(short* opt) {
	*opt = 5;
	asgd_start = imu_start = false;
	gscan_start = false;
	logging_start = true;
	control_start = false;
	ftsensor_start = true;
}

void opmode06(short* opt) {
	*opt = 6;
	asgd_start = imu_start = false;
	gscan_start = true;
	logging_start = true;
	control_start = false;
	ftsensor_start = false;
}

void opmode07(short* opt) {
	*opt = 7;
	asgd_start = imu_start = false;
	gscan_start = false;
	logging_start = false;
	control_start = false;
	ftsensor_start = false;
}

void opmode08(short* opt) {
	*opt = 8;
	asgd_start = imu_start = false;
	gscan_start = true;
	logging_start = true;
	control_start = true;
	ftsensor_start = false;
}

//////////////////////////////////////////\/////////\/
// INTERFACE DE CONTROLE EXO-TAU  /       /\     ////\
// EESC-USP                      / _____ ___  ___  //|
// RehabLab                     /  | |  | . \/   \  /|
// *Copyright 2021-2026* \//// //  | |   \ \   |_|  /|
//\///////////////////////\// //// \_'_/\_`_/__|   ///
///\///////////////////////\ //////////////////\/////\