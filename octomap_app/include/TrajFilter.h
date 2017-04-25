#ifndef TRAJFILTER_H
#define TRAJFILTER_H
#include "param.h"
class TrajFilter
{
public:
	TrajFilter(void);

	~TrajFilter(void);

	void posFilter(
		OUTPUT_PARAM&	dst, 
		unsigned long int	loc_time);

private:

	int calTimeDiff(
		TRAJ			t_current,
		TRAJ			t_early);

	float calSpeed(
		TRAJ			t_current, 
		TRAJ			t_early);

	float calTrajMeanSpeed();

private:
	vector<TRAJ>		m_traj;
	
};
#endif
