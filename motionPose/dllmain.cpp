#include "logging.h"

const char* logConfigFileName = "logging.conf";

const char* logConfigDefault =
"* GLOBAL:\n"
"	FORMAT = \"[%level] %datetime{%Y-%M-%d %H:%m:%s}: %msg\"\n"
"	FILENAME = \"driver_motionpose.log\"\n"
"	ENABLED = true\n"
"	TO_FILE = true\n"
"	TO_STANDARD_OUTPUT = true\n"
"	MAX_LOG_FILE_SIZE = 2097152 ## 2MB\n"
"* TRACE:\n"
"	ENABLED = true\n"
"* DEBUG:\n"
"	ENABLED = true\n";

INITIALIZE_EASYLOGGINGPP

void init_logging()
{
	el::Loggers::addFlag(el::LoggingFlag::DisableApplicationAbortOnFatalLog);
	el::Configurations conf(logConfigFileName);
	conf.parseFromText(logConfigDefault);
	//conf.parseFromFile(logConfigFileName);
	conf.setRemainingToDefault();
	el::Loggers::reconfigureAllLoggers(conf);	
}

BOOL APIENTRY DllMain(HMODULE hinstDLL, DWORD fdwReason, LPVOID lpvReserved)
{
	switch (fdwReason)
	{
	case DLL_PROCESS_ATTACH:
		init_logging();
		LOG(INFO) << "|========================================================================================|";
		LOG(INFO) << "motionPose dll loaded...";
		LOG(TRACE) << "Trace messages enabled.";
		LOG(DEBUG) << "Debug messages enabled.";
		break;
	case DLL_THREAD_ATTACH:
	case DLL_THREAD_DETACH:
	case DLL_PROCESS_DETACH:
		break;
	};
	return TRUE;
}