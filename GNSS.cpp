#include"GNSS.h"
using namespace std;
void ReadGNSSData(char* FileName, map<double, _GNSS>& mapGnss)
{
	FILE* fp_gnss = NULL;
	char buffer[256];
	_GNSS data;
	fopen_s(&fp_gnss, FileName, "r");
	if (fp_gnss == NULL)
	{
		printf("Open Error!");
		return;
	}
	////先读取前两行
	fgets(buffer, 256, fp_gnss); //printf("%s", buffer);
	fgets(buffer, 256, fp_gnss);// printf("%s", buffer);

	while (feof(fp_gnss) == 0)
	{
		fgets(buffer, 256, fp_gnss);
		//printf("%s", buffer);
		sscanf_s(buffer, "%hd%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf",
			&data.gt.Week, &data.gt.SecOfWeek, &data.pos[0], &data.pos[1], &data.pos[2],
			&data.stdpos[0], &data.stdpos[1], &data.stdpos[2],
			&data.vel[0], &data.vel[1], &data.vel[2],
			&data.stdvel[0], &data.stdvel[1], &data.stdvel[2]);
		/*printf("%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n", data[i].time, data[i].pos.arrayblh[0], data[i].pos.arrayblh[1], data[i].pos.arrayblh[2],
			data[i].stdpos[0], data[i].stdpos[1], data[i].stdpos[2],
			data[i].vel[0], data[i].vel[1], data[i].vel[2],
			data[i].stdvel[0], data[i].stdvel[1], data[i].stdvel[2]);*/
		data.stdpos[2] = data.stdpos[2] * (-1);
		data.vel[2] = data.vel[2] * (-1);
		data.stdvel[2] = data.stdvel[2] * (-1);
		data.pos[0] = deg2rad(data.pos[0]);
		data.pos[1] = deg2rad(data.pos[1]);
		mapGnss[data.gt.SecOfWeek] = data;
	}
	fclose(fp_gnss);

}
