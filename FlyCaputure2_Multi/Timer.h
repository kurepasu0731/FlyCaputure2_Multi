#ifndef TIMER_H
#define TIMER_H

#include <Windows.h>
#include <iostream>

class Timer
{
public:
	Timer()
	{
		// 初期化
		memset(&nFreq,   0x00, sizeof nFreq);
		memset(&nBefore, 0x00, sizeof nBefore);
		memset(&nAfter,  0x00, sizeof nAfter);
		duration = 0;
		restart();
	}
public:
	/// 計測開始
	void restart()
	{
		QueryPerformanceFrequency(&nFreq);
		QueryPerformanceCounter(&nBefore);
	}
	/// restartからの秒数を返す
	DWORD elapsed()
	{
		QueryPerformanceCounter(&nAfter);

		duration = (DWORD)((nAfter.QuadPart - nBefore.QuadPart) * 1000 / nFreq.QuadPart);
		std::cout << duration << " [ms]" << std::endl;
		return duration;
	}
private:
	/// 処理時間
	DWORD duration;
	/// 時間計測に必要な変数
	LARGE_INTEGER nFreq, nBefore, nAfter;
};

#endif
