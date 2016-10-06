#ifndef TIMER_H
#define TIMER_H

#include <Windows.h>
#include <iostream>

class Timer
{
public:
	Timer()
	{
		// ������
		memset(&nFreq,   0x00, sizeof nFreq);
		memset(&nBefore, 0x00, sizeof nBefore);
		memset(&nAfter,  0x00, sizeof nAfter);
		duration = 0;
		restart();
	}
public:
	/// �v���J�n
	void restart()
	{
		QueryPerformanceFrequency(&nFreq);
		QueryPerformanceCounter(&nBefore);
	}
	/// restart����̕b����Ԃ�
	DWORD elapsed()
	{
		QueryPerformanceCounter(&nAfter);

		duration = (DWORD)((nAfter.QuadPart - nBefore.QuadPart) * 1000 / nFreq.QuadPart);
		std::cout << duration << " [ms]" << std::endl;
		return duration;
	}
private:
	/// ��������
	DWORD duration;
	/// ���Ԍv���ɕK�v�ȕϐ�
	LARGE_INTEGER nFreq, nBefore, nAfter;
};

#endif
