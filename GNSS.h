#pragma once
#ifndef GNSS_H

#include<iostream>
#include<stdio.h>
#include<fstream>
#include<string>
#include<map>
#include"Coor.h"
#include"Time.h"

struct _GNSS
{
	GPST gt;

	double pos[3];
	double stdpos[3];
	double vel[3];
	double stdvel[3];
	_GNSS()
	{
		//time = 0.0;
		for (int i = 0; i < 3; i++)
		{
			pos[i]= stdpos[i] = vel[i] = stdvel[i] = 0.0;
		}
	}
};

void ReadGNSSData(char* FileName, std:: map<double, _GNSS>& mapGnss);
#endif // !GNSS_H
