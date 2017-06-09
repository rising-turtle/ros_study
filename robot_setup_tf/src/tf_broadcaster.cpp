#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <fstream>
#include <pthread.h>
#include <stdlib.h>

#define w 10

using namespace std;

pthread_mutex_t msg_mut_;
pthread_mutex_t ack_mut_;

int g_idx=-1;
char g_msg;
int g_ext=0;
int g_ack=0;
char Q_buff[15];

void send(char m, int idx, int ext)
{
	pthread_mutex_lock(&msg_mut_); 
		g_idx=idx;
		g_msg=m;
		g_ext=ext;
	pthread_mutex_lock(&msg_mut_); 
}

void send_ack(int ack)
{
	pthread_mutex_lock(&ack_mut_); 
		g_ack=ack;
	pthread_mutex_lock(&ack_mut_); 
}

void recv_ack(int& ack)
{
	ack=g_ack;
}

void recv(char& m, int& idx, int& ext)
{
	idx=g_idx;
	m=g_msg;
	ext=g_ext;
}
void *P_proc(void *ptr)
{
	int idx=0;
	char m='a';
	int loc_ack=0;
	int loc_exit=0;
	while (1)
	{
		cout<<"\n1.1\n";		
		for (int i=0;i<w; i++)
		{
			double rnd=rand()/10000000000;			
			cout<<"\nrand="<<rnd*15<<"\n";		
			idx=rnd*15;//consider this a new msg received from P	
			cout<<"\nsent at idx="<<idx<<"\n";					
			if (idx == 14)
			{
				send(m,idx,1);
				loc_exit=1;
				cout<<"\nExiting P\n";							
				break;
			}else
			{
				send(m,idx,0);
			}
			cout<<"\n1.3\n";		
		}
		cout<<"\n1.4\n";		
		recv_ack(loc_ack);
		if ((loc_ack == 0) || (loc_exit == 1))
		{
			cout<<"Error";
			break;
		}
	}
}

void *Q_proc(void *ptr)
{
	int k=0;
	char buff[w];

	for(int i=0;i<w;i++)
		buff[i]='0';

	int r=0;
	char m;
	int idx;
	int ext=0;	

	while(1)
	{
		//cout<<"\n2.1\n";		
		for(int j=0;j<w;j++)
		{
			//cout<<"\n2.2\n";		
			recv(m,idx,ext);
			//cout<<"\n*"<<idx%w<<"*\n";
			buff[idx%w]=m;
			while( buff[k]=='0')
			{
				//cout<<"\nr="<<r<<"\n";
				//cout<<"\nk="<<k<<"\n";
				//cout<<"\nw="<<w<<"\n";
				//cout<<"\n**"<<k+(r*w)<<"**\n";
				Q_buff[k+(r*w)]=m;
				buff[k]='0';
				k++;
			}			
			//cout<<"\n2.3\n";		
			if (ext == 1)
			{
				cout<<"\nExit recvd by Q\n";							
				break;
			}
		}
		//cout<<"\n2.4\n";		
		k=0;
		r++;
		send_ack(1);
		if (ext == 1)
		{
			cout<<"\nExiting Q\n";			
			break;
		}
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "robot_tf_broadcaster");
	ros::NodeHandle n;

	cout<<"\n0\n";
	ros::Rate r(100);
	pthread_mutex_init(&msg_mut_, NULL);
	pthread_mutex_init(&ack_mut_, NULL);

	// spawn another thread
	pthread_t thread_P;

	cout<<"\n1\n";	
	int rc1;
	/*Create independent threads each of which will execute functionC */
	if( (rc1=pthread_create( &thread_P, NULL, &P_proc, NULL)) )
   	{
		printf("Thread creation failed: %d\n", rc1);
	}

	cout<<"\n2\n";
	// spawn another thread
	pthread_t thread_Q;
	
	int rc2;
	/*Create independent threads each of which will execute functionC */
	if( (rc2=pthread_create( &thread_Q, NULL, &Q_proc, NULL)) )
   	{
		printf("Thread creation failed: %d\n", rc2);
	}

	while((n.ok()) && (g_ext == 0))
	{
		usleep(10000); // sleep 10 ms
	}	
	cout<<"\n3\n";
	//r.sleep();
}
