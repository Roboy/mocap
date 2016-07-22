/*
Chris Cummings
This is a heavily rewritten version of the core part of raspivid (copy right below)
and the work done by Pierre Raus at http://raufast.org/download/camcv_vid0.c to get
the camera feeding into opencv. It wraps up the camera system in a simple
StartCamera, StopCamera and callback to read data from the feed.
*/


/*
Copyright (c) 2013, Broadcom Europe Ltd
Copyright (c) 2013, James Hughes
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "camera.h"
#include <stdio.h>

// Standard port setting for the camera component
#define MMAL_CAMERA_PREVIEW_PORT 0
#define MMAL_CAMERA_VIDEO_PORT 1
#define MMAL_CAMERA_CAPTURE_PORT 2

static CCamera* GCamera = NULL;

CCamera* StartCamera(int width, int height, int framerate, CameraCBFunction callback)
{
	//can't create more than one camera
	if(GCamera != NULL)
	{
		printf("Can't create more than one camera\n");
		return NULL;
	}

	//create and attempt to initialize the camera
	GCamera = new CCamera();
	if(!GCamera->Init(width,height,framerate,callback))
	{
		//failed so clean up
		printf("Camera init failed\n");
		delete GCamera;
		GCamera = NULL;
	}
	return GCamera;
}

void StopCamera()
{
	if(GCamera)
	{
		GCamera->Release();
		delete GCamera;
		GCamera = NULL;
	}
}

CCamera::CCamera()
{

}

CCamera::~CCamera()
{

}

void CCamera::CameraControlCallback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
	GCamera->OnCameraControlCallback(port,buffer);
}
void CCamera::VideoBufferCallback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
	GCamera->OnVideoBufferCallback(port,buffer);
}

bool CCamera::Init(int width, int height, int framerate, CameraCBFunction callback)
{
	//init broadcom host - QUESTION: can this be called more than once??
	bcm_host_init();

	//store basic parameters
	Width = width;       
	Height = height;
	FrameRate = framerate;
	Callback = callback;

	// Set up the camera_parameters to default
	raspicamcontrol_set_defaults(&CameraParameters);

	MMAL_COMPONENT_T *camera = 0;
	MMAL_ES_FORMAT_T *format;
	MMAL_PORT_T *preview_port = NULL, *video_port = NULL, *still_port = NULL;
	MMAL_STATUS_T status;

	//create the camera component
	status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &camera);
	if (status != MMAL_SUCCESS)
	{
		printf("Failed to create camera component\n");
		return false;
	}

	//check we have output ports
	if (!camera->output_num)
	{
		printf("Camera doesn't have output ports");
		mmal_component_destroy(camera);
		return false;
	}

	//get the 3 ports
	preview_port = camera->output[MMAL_CAMERA_PREVIEW_PORT];
	video_port = camera->output[MMAL_CAMERA_VIDEO_PORT];
	still_port = camera->output[MMAL_CAMERA_CAPTURE_PORT];

	// Enable the camera, and tell it its control callback function
	status = mmal_port_enable(camera->control, CameraControlCallback);
	if (status != MMAL_SUCCESS)
	{
		printf("Unable to enable control port : error %d", status);
		mmal_component_destroy(camera);
		return false;
	}

	//  set up the camera configuration
	{
		MMAL_PARAMETER_CAMERA_CONFIG_T cam_config;
		cam_config.hdr.id = MMAL_PARAMETER_CAMERA_CONFIG;
		cam_config.hdr.size = sizeof(cam_config);
		cam_config.max_stills_w = Width;
		cam_config.max_stills_h = Height;
		cam_config.stills_yuv422 = 0;
		cam_config.one_shot_stills = 0;
		cam_config.max_preview_video_w = Width;
		cam_config.max_preview_video_h = Height;
		cam_config.num_preview_video_frames = 3;
		cam_config.stills_capture_circular_buffer_height = 0;
		cam_config.fast_preview_resume = 0;
		cam_config.use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RESET_STC;
		mmal_port_parameter_set(camera->control, &cam_config.hdr);
	}

	// setup preview port format - QUESTION: Needed if we aren't using preview?
	format = preview_port->format;
	format->encoding = MMAL_ENCODING_OPAQUE;
	format->encoding_variant = MMAL_ENCODING_I420;
	format->es->video.width = Width;
	format->es->video.height = Height;
	format->es->video.crop.x = 0;
	format->es->video.crop.y = 0;
	format->es->video.crop.width = Width;
	format->es->video.crop.height = Height;
	format->es->video.frame_rate.num = FrameRate;
	format->es->video.frame_rate.den = 1;
	status = mmal_port_format_commit(preview_port);
	if (status != MMAL_SUCCESS)
	{
		printf("Couldn't set preview port format : error %d", status);
		mmal_component_destroy(camera);
		return false;
	}

	//setup video port format
	format = video_port->format;
	format->encoding = MMAL_ENCODING_I420;
	format->encoding_variant = MMAL_ENCODING_I420; //not opaque, as we want to read it!
	format->es->video.width = Width;
	format->es->video.height = Height;
	format->es->video.crop.x = 0;
	format->es->video.crop.y = 0;
	format->es->video.crop.width = Width;
	format->es->video.crop.height = Height;
	format->es->video.frame_rate.num = FrameRate;
	format->es->video.frame_rate.den = 1;
	status = mmal_port_format_commit(video_port);
	if (status != MMAL_SUCCESS)
	{
		printf("Couldn't set video port format : error %d", status);
		mmal_component_destroy(camera);
		return false;
	}

	//setup still port format
	format = still_port->format;
	format->encoding = MMAL_ENCODING_OPAQUE;
	format->encoding_variant = MMAL_ENCODING_I420;
	format->es->video.width = Width;
	format->es->video.height = Height;
	format->es->video.crop.x = 0;
	format->es->video.crop.y = 0;
	format->es->video.crop.width = Width;
	format->es->video.crop.height = Height;
	format->es->video.frame_rate.num = 1;
	format->es->video.frame_rate.den = 1;
	status = mmal_port_format_commit(still_port);
	if (status != MMAL_SUCCESS)
	{
		printf("Couldn't set still port format : error %d", status);
		mmal_component_destroy(camera);
		return false;
	}

	//setup video port buffer and a pool to hold them
	video_port->buffer_num = 3;
	video_port->buffer_size = video_port->buffer_size_recommended;
	MMAL_POOL_T* video_buffer_pool;
	printf("Creating video port pool with %d buffers of size %d\n", video_port->buffer_num, video_port->buffer_size);
	video_buffer_pool = mmal_port_pool_create(video_port, video_port->buffer_num, video_port->buffer_size);
	if (!video_buffer_pool)
	{
		printf("Couldn't create video buffer pool\n");
		mmal_component_destroy(camera);
		return false;	
	}

	//enable the camera
	status = mmal_component_enable(camera);
	if (status != MMAL_SUCCESS)
	{
		printf("Couldn't enable camera\n");
		mmal_port_pool_destroy(video_port,video_buffer_pool);
		mmal_component_destroy(camera);
		return false;	
	}

	//apply all camera parameters
	raspicamcontrol_set_all_parameters(camera, &CameraParameters);

	//setup the video buffer callback
	status = mmal_port_enable(video_port, VideoBufferCallback);
	if (status != MMAL_SUCCESS)
	{
		printf("Failed to set video buffer callback\n");
		mmal_port_pool_destroy(video_port,video_buffer_pool);
		mmal_component_destroy(camera);
		return false;	
	}

	//send all the buffers in our pool to the video port ready for use
	{
		int num = mmal_queue_length(video_buffer_pool->queue);
		int q;
		for (q=0;q<num;q++)
		{
			MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(video_buffer_pool->queue);
			if (!buffer)
				printf("Unable to get a required buffer %d from pool queue", q);
			if (mmal_port_send_buffer(video_port, buffer)!= MMAL_SUCCESS)
				printf("Unable to send a buffer to encoder output port (%d)", q);
			printf("Sent buffer %d to video port\n");
		}
	}

	//begin capture
	if (mmal_port_parameter_set_boolean(video_port, MMAL_PARAMETER_CAPTURE, 1) != MMAL_SUCCESS)
	{
		printf("Failed to start capture\n");
		mmal_port_pool_destroy(video_port,video_buffer_pool);
		mmal_component_destroy(camera);
		return false;	
	}

	//store created info
	CameraComponent = camera;
	BufferPool = video_buffer_pool;

	//return success
	printf("Camera successfully created\n");
	return true;
}

void CCamera::Release()
{
	printf("Shutting down camera\n");
	mmal_port_disable(CameraComponent->output[MMAL_CAMERA_VIDEO_PORT]);
	mmal_component_disable(CameraComponent);
	mmal_port_pool_destroy(CameraComponent->output[MMAL_CAMERA_VIDEO_PORT],BufferPool);
	mmal_component_destroy(CameraComponent);
}

void CCamera::OnCameraControlCallback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
	printf("Camera control callback\n");
}

void CCamera::OnVideoBufferCallback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
	//check if buffer has data in
	if(buffer->length)
	{
		//got data so lock the buffer, call the callback so the application can use it, then unlock
		mmal_buffer_header_mem_lock(buffer);
		Callback(this,buffer->data,buffer->length);
		mmal_buffer_header_mem_unlock(buffer);
	}
	
	// release buffer back to the pool
	mmal_buffer_header_release(buffer);

	// and send one back to the port (if still open)
	if (port->is_enabled)
	{
		MMAL_STATUS_T status;
		MMAL_BUFFER_HEADER_T *new_buffer;
		new_buffer = mmal_queue_get(BufferPool->queue);
		if (new_buffer)
			status = mmal_port_send_buffer(port, new_buffer);
		if (!new_buffer || status != MMAL_SUCCESS)
			printf("Unable to return a buffer to the video port\n");
	}
}

