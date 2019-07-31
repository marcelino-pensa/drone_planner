/*
 * WIN32 Events for POSIX
 * Author: Mahmoud Al-Qudsi <mqudsi@neosmart.net>
 * Copyright (C) 2011 - 2015 by NeoSmart Technologies
 * This code is released under the terms of the MIT License
*/

#pragma once

#if defined(_WIN32) && !defined(CreateEvent)
#error Must include Windows.h prior to including pevents.h!
#endif
#ifndef WAIT_TIMEOUT
#include <errno.h>
#define WAIT_TIMEOUT ETIMEDOUT
#endif

#include <stdint.h>

namespace pevents
{
	//Type declarations
	struct neosmart_event_t_;
	typedef neosmart_event_t_ * event_t;

	// constant definitions
	const bool MANUAL_RESET     = true;
	const bool AUTO_RESET       = false;
	const bool INIT_STATE_TRUE  = true;
	const bool INIT_STATE_FALSE = false;
	
	//WIN32-style functions
	event_t CreateEvent(bool manualReset = false, bool initialState = false);
	int DestroyEvent(event_t event);
	int WaitForEvent(event_t event, uint64_t milliseconds = -1);
	int SetEvent(event_t event);
	int ResetEvent(event_t event);
#ifdef WFMO
	int WaitForMultipleEvents(event_t *events, int count, bool waitAll, uint64_t milliseconds);
	int WaitForMultipleEvents(event_t *events, int count, bool waitAll, uint64_t milliseconds, int &index);
#endif
#ifdef PULSE
	int PulseEvent(event_t event);
#endif
    
	//POSIX-style functions
	//TBD
}
