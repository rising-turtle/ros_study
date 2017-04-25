#include "TrajFilter.h"

TrajFilter::TrajFilter(void)
{
}

TrajFilter::~TrajFilter(void)
{
}
//return time interval in ms
int TrajFilter::calTimeDiff(TRAJ t_current, TRAJ t_early)
{
	/*int diff_hour = t_current.loc_time.wHour - t_early.loc_time.wHour;
	int diff_min = diff_hour * 60 + t_current.loc_time.wMinute - t_early.loc_time.wMinute;
	int diff_second = diff_min * 60 + t_current.loc_time.wSecond - t_early.loc_time.wSecond;
	int diff_ms = diff_second * 1000 + t_current.loc_time.wMilliseconds - t_early.loc_time.wMilliseconds;*/
	int diff_ms = t_current.cap_time - t_early.cap_time;
	return diff_ms;
}
float TrajFilter::calSpeed(TRAJ t_current, TRAJ t_early)
{
	float spd = 0;
	int inter_time = calTimeDiff(t_current, t_early);
	//float dis = abs(t_current.loc_x-t_early.loc_x) +
	//	abs(t_current.loc_y-t_early.loc_y) +
	//	abs(t_current.loc_z-t_early.loc_z);
	float dis = abs(t_current.loc_x-t_early.loc_x) +
		abs(t_current.loc_y-t_early.loc_y);	

	if (0>=inter_time)
	{
		spd = 0;
	}
	else
		spd = dis / inter_time * 1000;
	if (spd<MAX_SPEED*0.5 && dis<0.1)
	{
		spd = 0.1;
	}	
	return spd;
}
float TrajFilter::calTrajMeanSpeed()
{
	int nSize = m_traj.size();
	float spd = 0;
	float mean_spd = 0;
	int cnt = 0;
	float max_spd = 0;
	float min_spd = 99999;//remove maximum and minimum speed;
	for (int i=0; i<nSize; i++)
	{
		for(int j=i+1; j<nSize; j++)
		{
			spd = calSpeed(m_traj[j], m_traj[i]);
			if (spd>MAX_SPEED)
			{
				continue;
			}
			if (spd>max_spd)
			{
				max_spd = spd;
			}
			if (spd<min_spd)
			{
				min_spd = spd;
			}
			mean_spd += spd;
			cnt++;
		}
	}
	if (cnt>2)
	{
		mean_spd -= max_spd;
		mean_spd -= min_spd;
		mean_spd /= (cnt-2);
	}
	else
		mean_spd = 0;
	return mean_spd;
}
void TrajFilter::posFilter(OUTPUT_PARAM& dst, unsigned long int loc_time)
{
	if (!dst.bOK)
	{
		return;
	}
	TRAJ curr_traj;
	curr_traj.cap_time = loc_time;
	curr_traj.loc_x = dst.loc_x[0];
	curr_traj.loc_y = dst.loc_y[0];
	curr_traj.loc_z = dst.loc_z[0];
	curr_traj.v_x_prev = 0;
	curr_traj.v_y_prev = 0;
	curr_traj.v_z_prev = 0;
	//just add the position to trajectory, in the beginning;
	int size_traj = m_traj.size();
	if (size_traj)
	{
		int inter_t_ms = calTimeDiff(curr_traj, m_traj[size_traj-1]);
		curr_traj.v_x_prev = (curr_traj.loc_x - m_traj[size_traj-1].loc_x) / inter_t_ms * 1000;
		curr_traj.v_y_prev = (curr_traj.loc_y - m_traj[size_traj-1].loc_y) / inter_t_ms * 1000;
		curr_traj.v_z_prev = (curr_traj.loc_z - m_traj[size_traj-1].loc_z) / inter_t_ms * 1000;
	}
	if (size_traj<10)
	{		
		
		m_traj.push_back(curr_traj);
		cout<<"trajectory size: "<<m_traj.size()<<endl;
		return;
	}
	//restrict the size of m_traj within 10;
	if (m_traj.size()>=20)
	{
		m_traj.erase(m_traj.begin());
	}
	size_traj = m_traj.size();//update
	//int diff_hour = curr_traj.loc_time.wHour - m_traj[size_traj-1].loc_time.wHour;
	//int diff_min = diff_hour * 60 + curr_traj.loc_time.wMinute - m_traj[size_traj-1].loc_time.wMinute;
	int diff_time = curr_traj.cap_time - m_traj[size_traj-1].cap_time;
	if (diff_time>5*60*1000) //  the interval of time is too long (5 minutes), clear the history trajectory
	{
		m_traj.clear();			//clear history trajectory
		m_traj.push_back(curr_traj);	//add the new position
		return;
	}

	float traj_mean_spd = calTrajMeanSpeed();
	float adp_spd_thrd = max(traj_mean_spd * 1.5, 0.3);
	//judge the current position based on trajectory
	float speed = 0;
	int violate_cnt = 0;
	float mean_speed = 0;
	float dis = 0;
	for (int i=1; i<11; i++)
	{
		speed = calSpeed(curr_traj, m_traj[size_traj-i]);
		mean_speed += speed;
		if (speed>MAX_SPEED || speed>adp_spd_thrd)
		{
			violate_cnt++;
		}	

	}
	mean_speed = mean_speed / 10;

	if (violate_cnt>5)
	{
		dst.bOK = false;
		dst.bFiltered = true;
	}
	else if (violate_cnt>2 && mean_speed>traj_mean_spd*1.5)
	{
		dst.bOK = false;
		dst.bFiltered = true;
	}
	else
	{
		dst.bOK = true;
		m_traj.push_back(curr_traj);
	}

}