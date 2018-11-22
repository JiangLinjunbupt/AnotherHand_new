#include"OpenGL_Display.h"
#include <tchar.h>
//共享内存的相关定义
HANDLE hMapFile;
LPCTSTR pBuf;
#define BUF_SIZE 1024
TCHAR szName[] = TEXT("Global\\MyFileMappingObject");    //指向同一块共享内存的名字

int main(int argc,char** argv)
{
#pragma region SharedMemery
	hMapFile = CreateFileMapping(
		INVALID_HANDLE_VALUE,    // use paging file
		NULL,                    // default security
		PAGE_READWRITE,          // read/write access
		0,                       // maximum object size (high-order DWORD)
		BUF_SIZE,                // maximum object size (low-order DWORD)
		szName);                 // name of mapping object

	if (hMapFile == NULL)
	{
		_tprintf(TEXT("Could not create file mapping object (%d).\n"),
			GetLastError());
		return 1;
	}
	pBuf = (LPTSTR)MapViewOfFile(hMapFile,   // handle to map object
		FILE_MAP_ALL_ACCESS, // read/write permission
		0,
		0,
		BUF_SIZE);

	if (pBuf == NULL)
	{
		_tprintf(TEXT("Could not map view of file (%d).\n"),
			GetLastError());

		CloseHandle(hMapFile);

		return 1;
	}

	DS::GetSharedMemeryPtr = (float*)pBuf;
#pragma endregion SharedMemery

	Camera *camera = new Camera();
	HandModel *model = new HandModel(camera);
	model->Print_fingerLength();
	Worker *worker = new Worker(model);
	worker->load_target_joints();
	worker->load_target_vertices();
	worker->fetch_Input(7);


	DS::handmodel = model;
	DS::worker = worker;

	DS::init(argc, argv);
	DS::start();

	return 0;
}