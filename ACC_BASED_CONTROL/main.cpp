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

#include "SharedStructs.h" // ja inclui <stdio.h> / <thread> / <mutex> / <vector>
#if CAN_ENABLE
#include "XsensEpos.h"
#endif
#include "QpcLoopTimer.h" // ja inclui <windows.h>
#include <processthreadsapi.h>
#include <stdexcept>
#include <iostream>
#include <conio.h>

void Interface();
void readIMUs(ThrdStruct& data_struct);
void readFTSensor(ThrdStruct& data_struct);
void qASGD(ThrdStruct& data_struct);
void Controle(ThrdStruct& data_struct);
void Logging(ThrdStruct& data_struct);
void updateGains(ThrdStruct& data_struct);

// DEBUGGING DEFINES:
#define PRIORITY     0
#define ISREADY_WAIT 0
#define EXEC_TIME   50

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

// Global vars shared with Interface() funct
ThrdStruct imu_struct, asgd_struct, ftsensor_struct;
ThrdStruct control_struct, logging_struct, gscan_struct;
short imu_isready(false);
short asgd_isready(false);
short control_isready(false);
short logging_isready(false);
short ftsensor_isready(false);
short gscan_isready(false);

short imu_aborting(false);
short asgd_aborting(false);
short control_aborting(false);
short logging_aborting(false);
short ftsensor_aborting(false);
short gscan_aborting(false);

bool execution_end(false);
int  execution_time = EXEC_TIME;

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

#if CAN_ENABLE
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

	// Function Structs definition:
	{ // Apenas para colapsar e facilitar leitura do código

	// IMU Struct:
		imu_struct.sampletime_ = IMU_SMPLTM;
		imu_struct.param00_ = IMU_PRIORITY;
		imu_struct.param0A_ = &imu_isready;
		imu_struct.param1A_ = &imu_aborting;
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
	GLFWwindow* window = glfwCreateWindow(1820, 1080, "Project Hylonome -- D.ImGui [GLFW | OpenGL3]", NULL, NULL);
	if (window == NULL)
		return 1;
	glfwSetWindowPos(window, 0, 38);
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1); // Enable vsync

	// Setup Dear ImGui context
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO(); (void)io;
	//io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
	//io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

	// Setup Dear ImGui style
	ImGui::StyleColorsClassic();

	// Setup Platform/Renderer backends
	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init(glsl_version);

	// Our state
	bool show_demo_window = false;
	bool show_another_window = false;
	ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

	system("cls");
	/*
	do
	{
		system("cls");
		Interface();
		// Fire Threads:
		if (!imu_isready)      thr_imus = thread(readIMUs, imu_struct);
		if (!asgd_isready)     thr_qasgd = thread(qASGD, asgd_struct);
		if (!logging_isready)  thr_logging = thread(Logging, logging_struct);
		if (!control_isready)  thr_controle = thread(Controle, control_struct);
		if (!ftsensor_isready) thr_ftsensor = thread(readFTSensor, ftsensor_struct);
		if (!gscan_isready)    thr_gainscan = thread(updateGains, gscan_struct);

		// main waits while the threads execute thier tasks...
		if (thr_controle.joinable()) thr_controle.join();
		if (thr_logging.joinable()) thr_logging.join();
		if (thr_ftsensor.joinable()) thr_ftsensor.join();
		if (thr_gainscan.joinable()) thr_gainscan.join();
		if (thr_qasgd.joinable()) thr_qasgd.join();
		if (thr_imus.joinable()) thr_imus.join();

	} while (!execution_end);
	*/

	// Main loop
	while (!glfwWindowShouldClose(window) && !execution_end)
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

			ImGui::Begin("Hello, ExoTau!");                          // Create a window with string name...

			ImGui::Text(" //////////////////////////////////////////\\/////////\\/");
			ImGui::Text(" // INTERFACE DE CONTROLE EXO-TAU  /       /\\     ////\\");
			ImGui::Text(" // EESC-USP                      / _____ ___  ___  //|"     );
			ImGui::Text(" // RehabLab                     /  | |  | . \\/   \\  /|"   );
			ImGui::Text(" // *Copyright 2021-2026* \\//// //  | |   \\ \\   |_|  /|"  );
			ImGui::Text(" //\\///////////////////////\\// //// \\_'_/\\_`_/__|   ///" );
			ImGui::Text(" ///\\///////////////////////\\ //////////////////\\/////\\" );

			ImGui::Checkbox("Demo Window", &show_demo_window);      // Edit bools storing our window open/close state
			ImGui::Checkbox("Another Window", &show_another_window);

			ImGui::SliderFloat("float", &f, -5.0f, 5.0f);            // Edit 1 float using a slider
			ImGui::ColorEdit3("clear color", (float*)&clear_color); // Edit 3 floats representing a color

			if (ImGui::Button("Button"))                            // Buttons return true when clicked (most widgets return true when edited/activated)
				counter++;
			ImGui::SameLine();
			ImGui::Text("counter = %d", counter);

			ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
			ImGui::End();
		}

		// 3. Show another simple window.
		if (show_another_window)
		{
			ImGui::Begin("Another Window", &show_another_window);   // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)
			ImGui::Text("Hello from another window!");
			if (ImGui::Button("Close Me"))
				show_another_window = false;
			ImGui::End();
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
	cout << " Successful exit." << endl;
	// ImGui cleanup:
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();

	glfwDestroyWindow(window);
	glfwTerminate();
	return 0;
}

// Funcao de interface com o usuario:
void Interface()
{
	short option = 0;
	using namespace std;

	cout << " //////////////////////////////////////////\\/////////\\/" << endl;
	cout << " // INTERFACE DE CONTROLE EXO-TAU  /       /\\     ////\\" << endl;
	cout << " // EESC-USP                      / _____ ___  ___  //|" << endl;
	cout << " // RehabLab                     /  | |  | . \\/   \\  /|" << endl;
	cout << " // *Copyright 2021-2026* \\//// //  | |   \\ \\   |_|  /|" << endl;
	cout << " //\\///////////////////////\\// //// \\_'_/\\_`_/__|   ///" << endl;
	cout << " ///\\///////////////////////\\ //////////////////\\/////\\" << endl;

	cout << " Escolha uma opcao: \n";
	cout << " [01]: Controle com IMUs + F/T \n";
	cout << " [02]: Controle com IMUs \n";
	cout << " [03]: Controle 'IMU3 bypass' \n";
	cout << " [04]: Leitura IMUs \n";
	cout << " [05]: Leitura F/T  \n";
	cout << " [06]: Leitura Parametros  \n";
	cout << " [07]: Reset Falhas CAN \n";
	cout << " [08]: Controle sem IMUs (MF SEA) \n";
	cout << " Ou zero (0) para finalizar. \n ";
	cin >> option;

	switch (option)
	{
	case 1:
		// tudo roda:
		asgd_isready = imu_isready = false;
		gscan_isready = false;
		logging_isready = false;
		control_isready = false;
		ftsensor_isready = false;
		execution_end = false;
		break;
	case 2:
		asgd_isready = imu_isready = false;
		gscan_isready = false;
		logging_isready = false;
		control_isready = false;
		ftsensor_isready = true;
		execution_end = false;
		break;
	case IMUBYPASS:
		imu_isready = false;
		asgd_isready = false;
		gscan_isready = false;
		logging_isready = false;
		control_isready = false;
		ftsensor_isready = true;
		execution_end = false;
		break;
	case READIMUS:
		asgd_isready = imu_isready = false;
		gscan_isready = true;
		logging_isready = false;
		control_isready = true;
		ftsensor_isready = true;
		execution_end = false;
		break;
	case 5:
		asgd_isready = imu_isready = true;
		gscan_isready = true;
		logging_isready = false;
		control_isready = true;
		ftsensor_isready = false;
		execution_end = false;
		break;
	case 6:
		asgd_isready = imu_isready = true;
		gscan_isready = false;
		logging_isready = false;
		control_isready = true;
		ftsensor_isready = true;
		execution_end = false;
		break;
	case 7:
		imu_isready = true;
		asgd_isready = true;
		gscan_isready = true;
		logging_isready = true;
		control_isready = true;
		ftsensor_isready = true;
		execution_end = false;
#if CAN_ENABLE
		ResetRedeCan();
		HabilitaEixo(2);
		DesabilitaEixo(2);
#endif
		break;
	case 8:
		asgd_isready = imu_isready = true;
		gscan_isready = false;
		logging_isready = false;
		control_isready = false;
		ftsensor_isready = true;
		execution_end = false;
		break;
	case 0:
	default:
		imu_isready = true;
		asgd_isready = true;
		gscan_isready = true;
		logging_isready = true;
		control_isready = true;
		ftsensor_isready = true;
		execution_end = true;
		break;
	}

	imu_struct.param39_ = option;
	asgd_struct.param39_ = option;
	control_struct.param39_ = option;
	logging_struct.param39_ = option;

	if (!execution_end && option != 7) {
		cout << "\n Defina o tempo de execucao em segundos: ";
		cin >> execution_time;
		imu_struct.exectime_ = execution_time;
		asgd_struct.exectime_ = execution_time;
		gscan_struct.exectime_ = execution_time;
		control_struct.exectime_ = execution_time;
		logging_struct.exectime_ = execution_time;
		ftsensor_struct.exectime_ = execution_time;

		// Escolher tipo de controle:
		system("cls");
		if (!control_isready) {
			cout << " Escolha uma controlador: \n";
			cout << " [01]: Acc-based Transparency (PD) \n";
			cout << " [02]: Acc-based Transparency Z(0) \n";
			cout << " [03]: Acc-based Transparency (LS) \n";
			cout << " [04]: Int-based Transparency (F/T)\n";
			cout << " [05]: Markoviano Impedancia \n";
			cout << " [06]: Markoviano Torque \n";
			cout << " [07]: Markoviano Z(0) + RUIDO \n";
			cout << " [08]: Malha Fechada SEA (MF SEA) \n";
			cout << " [33]: GyroscopeX 'IMU3 bypass' \n";
			cout << " ";
			cin >> option;
			control_struct.param01_ = option;
			logging_struct.param10_ = option; // to logging the control option
			ftsensor_struct.param01_ = true; // F/T filtering on/off
		}
		cout << " Iniciando Threads..." << endl;
	}
}


//////////////////////////////////////////\/////////\/
// INTERFACE DE CONTROLE EXO-TAU  /       /\     ////\
// EESC-USP                      / _____ ___  ___  //|
// RehabLab                     /  | |  | . \/   \  /|
// *Copyright 2021-2026* \//// //  | |   \ \   |_|  /|
//\///////////////////////\// //// \_'_/\_`_/__|   ///
///\///////////////////////\ //////////////////\/////\