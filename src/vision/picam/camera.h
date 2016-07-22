#pragma once

#include "mmalincludes.h"
#include "cameracontrol.h"

class CCamera;

typedef void (*CameraCBFunction)(CCamera* cam, const void* buffer, int buffer_length);

class CCamera
{
public:

private:
	CCamera();
	~CCamera();

	bool Init(int width, int height, int framerate, CameraCBFunction callback);
	void Release();
	void OnCameraControlCallback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer);
	void OnVideoBufferCallback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer);
	static void CameraControlCallback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer);
	static void VideoBufferCallback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer);

	int							Width;
	int							Height;
	int							FrameRate;
	CameraCBFunction			Callback;
	RASPICAM_CAMERA_PARAMETERS	CameraParameters;
	MMAL_COMPONENT_T*			CameraComponent;    
	MMAL_POOL_T*				BufferPool;

	friend CCamera* StartCamera(int width, int height, int framerate, CameraCBFunction callback);
	friend void StopCamera();
};

CCamera* StartCamera(int width, int height, int framerate, CameraCBFunction callback);
void StopCamera();