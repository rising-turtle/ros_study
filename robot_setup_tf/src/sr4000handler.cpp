#include "sr4000handler.h"
#include <iostream>
#include <string>
#include <stdio.h>
#include <fstream>
#include <sstream>

#define TAB_DELIM_INPUT false

using namespace std;
using namespace cv;

//*********************************************************************************
sr4000handler::sr4000handler(std::string obs_path_input):
queue_index_xyz(-1), initialized_xyz(false), obs_no(0)
{	
	obs_path=obs_path_input;
}
///////////////////////////////////////////////////////////////////////////////////
inline bool fpExists(const std::string& name){
	if (FILE *file = fopen(name.c_str(),"r")){
		fclose(file);
		return true;
	}else {
		return false;
	}   
}
///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
void LOGMAT1(cv::Mat a)
{
	for (int i=0;i<a.rows;i++)
	{
		std::string tmp;
		for (int j=0;j<a.cols;j++)
		{
			tmp=tmp+std::to_string((long double)a.at<float>(i,j));
			tmp=tmp+" ";
		}
		//LOGD<<tmp;
	}
}
/////////////////////////////////////////////////////////////////////////////////////////
void sr4000handler::start_capture_dataDS()
{
	bool fp_exists=true;
	while(fp_exists)
	{
		sr4000data data_obj;
		getXYZObs(data_obj, fp_exists); //this function increments the file index internally
		if (fp_exists)
		{
			cv::Mat den_image=denoiseImage(data_obj.mat_img);
			data_obj.den_norm_img=normalizeImage(den_image); //denoise and normalize image since it is almost always required in this form
			data_queue.push_back(data_obj);
		}
	}
}
//****************************************************************************
vector<string> extract_lines(const std::string& file_name, unsigned int st_index, unsigned int end_index, bool &fp_exists, string &last_line)
{
	vector<std::string> extracted_lines ;

	ifstream file(file_name);
	if (file.good())
	{
		fp_exists=true;
	}else
	{
		fp_exists=false;
		return extracted_lines ;
	}
	
	string line;
	unsigned int ctr=1;
	while( std::getline( file, line ) ) 
	{
		if ((ctr >= st_index) && (ctr <= end_index))
		{
			extracted_lines.push_back(line);
		}

		if (line.size() > 1)
		{
			last_line=line;
		}
		ctr++;
	}
	//cout<<"\n*** "<<last_line<<" ***\n";
	return extracted_lines ;
}
//**********************************************************************************
void sr4000handler::getXYZObs(sr4000data &sd, bool &fp_exists)
{
	queue_index_xyz++;
	char buff[100];
	sprintf(buff,"%s/d1_%04d.dat",obs_path.c_str(),queue_index_xyz+1);
	string target_scan_path=buff;

	int j=0;

	try
	{
		
		int k=437;//144*3+1;
		cout<<target_scan_path<<"\n";
		string last_line;
		vector<std::string> extracted_lines = extract_lines(target_scan_path,k,k+143,fp_exists,last_line) ;
		
		if (fp_exists)
		{
			sd.mat_img = Mat::zeros(cv::Size(176,144), CV_32S);
			sd.z = Mat::zeros(cv::Size(176,144), CV_32F);
			sd.x = Mat::zeros(cv::Size(176,144), CV_32F);
			sd.y = Mat::zeros(cv::Size(176,144), CV_32F);
			sd.confidence_map = Mat::zeros(cv::Size(176,144), CV_32F);
		}else
		{
			if (queue_index_xyz<=0)
			{
				initialized_xyz=false;
			}
			queue_index_xyz--;
			return;
		}

		assert(extracted_lines.size() == 144);

		for(unsigned int ctr=0;ctr<extracted_lines.size();ctr++)
		{
			stringstream reader(extracted_lines.at(ctr));
			string extracted="garbage";
			bool end=false;
			while (end == false)
			{
				if (TAB_DELIM_INPUT == true)
					getline(reader,extracted,'\t');
				else
					getline(reader,extracted,' ');

				if( extracted.size() > 0)
				{
					//int temp=atoi(extracted.c_str());
					//cout<<temp<<"***"<<endl;
					//cout<<ctr<<" "<<j<<endl;
					sd.mat_img.at<int>(ctr,j)=atoi(extracted.c_str());
					j++;
					if (j > 175)
					{
						j=0;
						end=true;
					}
				}
			}
		}
		
		////////////////////////////////////////////////////////////////
		//cout<<target_scan_path<<"\n";
		int kz=2;//starting line index, 1 is the first line
		vector<std::string> extracted_lines_z = extract_lines(target_scan_path,kz,kz+143,fp_exists,last_line) ;
		int kx=kz+144+1;
		vector<std::string> extracted_lines_x = extract_lines(target_scan_path,kx,kx+143,fp_exists,last_line);
		int ky=kx+144+1;
		vector<std::string> extracted_lines_y = extract_lines(target_scan_path,ky,ky+143,fp_exists,last_line);			
		int kcon=ky+(2*144)+2;
		vector<std::string> extracted_lines_con = extract_lines(target_scan_path,kcon,kcon+143,fp_exists,last_line);

		assert(extracted_lines_z.size() == 144);
		assert(extracted_lines_x.size() == 144);
		assert(extracted_lines_y.size() == 144);
		assert(extracted_lines_con.size() == 144);
		
		j=0;
		for(unsigned int ctr=0;ctr<extracted_lines_z.size();ctr++)
		{
			stringstream readerz(extracted_lines_z.at(ctr));
			string extractedz="garbage";
			bool end=false;
			while (end == false)
			{
				if (TAB_DELIM_INPUT == true)
					getline(readerz,extractedz,'\t');
				else
					getline(readerz,extractedz,' ');					
				
				if (extractedz.size() > 0)
				{
					//cout<<ctr<<" "<<j<<endl;
					sd.z.at<float>(ctr,j)=(float)atof(extractedz.c_str());
					j++;

					if (j > 175)
					{
						j=0;
						end=true;
					}
				}
			}
		}

		//LOGMAT1(sd.z);
		j=0;
		for(unsigned int ctr=0;ctr<extracted_lines_x.size();ctr++)
		{
			stringstream readerx(extracted_lines_x.at(ctr));
			string extractedx="garbage";
			bool end=false;
			while (end == false)
			{
				if (TAB_DELIM_INPUT == true)
					getline(readerx,extractedx,'\t');
				else
					getline(readerx,extractedx,' ');

				if (extractedx.size() > 0) 
				{
					//cout<<ctr<<" "<<j<<endl;
					sd.x.at<float>(ctr,j)=(float)atof(extractedx.c_str());
					j++;

					if (j > 175)
					{
						j=0;
						end=true;
					}
				}
			}
		}

		j=0;
		for(unsigned int ctr=0;ctr<extracted_lines_y.size();ctr++)
		{
			stringstream readery(extracted_lines_y.at(ctr));
			string extractedy="garbage";
			bool end=false;
			while (end == false)
			{
				if (TAB_DELIM_INPUT == true)
					getline(readery,extractedy,'\t');
				else
					getline(readery,extractedy,' ');

				if (extractedy.size() > 0)
				{
					sd.y.at<float>(ctr,j)=(float)atof(extractedy.c_str());
					j++;

					if (j > 175)
					{
						j=0;
						end=true;
					}
				}
			}
		}
		
		if (extracted_lines_con.size() > 0)
		{
			j=0;
			for(unsigned int ctr=0;ctr<extracted_lines_con.size();ctr++)
			{
				stringstream readercon(extracted_lines_con.at(ctr));
				string extractedcon="garbage";
				bool end=false;
				while (end == false)
				{
					if (TAB_DELIM_INPUT == true)
						getline(readercon,extractedcon,'\t');
					else
						getline(readercon,extractedcon,' ');

					if (extractedcon.size() > 0)
					{
						//cout<<ctr<<" "<<j<<endl;
						sd.confidence_map.at<float>(ctr,j)=atoi(extractedcon.c_str());
						j++;

						if (j > 175)
						{
							j=0;
							end=true;
						}
					}
				}
			}
		}

		stringstream reader(last_line);
		string extracted="";
		while (true)
		{
			if (TAB_DELIM_INPUT == true)
			getline(reader,extracted,'\t');
				else
			getline(reader,extracted,' ');
			if (extracted.size() > 0)
			{
				sd.timestamp=atoi(extracted.c_str());
				break;
			}
		}
		//imwrite( "C:\\Users\\Muhammad\\Desktop\\test.png", img);
		initialized_xyz=true;
	}catch(std::exception &e)
	{
		printf("Exception in SR4000 read thread::sr4000handler::getXYZObs %s\n",e.what());
		initialized_xyz=false;
		queue_index_xyz--;
		fp_exists=false;
	}catch (...)
	{
		printf("Unknown exception in SR4000 read thread::sr4000handler::getXYZObs");
		initialized_xyz=false;
		queue_index_xyz--;
		fp_exists=false;
	}
	//matlab equiv
	//function [x,y,z,im,varargout]=read_xyz_sr4000_test(scan_file_name_prefix,my_k)
}
///////////////////////////////////////////////////////////////////////////////////
size_t sr4000handler::getDataQueueSize()
{
	return data_queue.size();
}
///////////////////////////////////////////////////////////////////////////////////
cv::Mat sr4000handler::denoiseImage(cv::Mat input)
{
	Mat den_img = Mat::zeros(cv::Size(input.cols,input.rows), CV_32S);

	int maximum=-100000;
	for(auto k=0;k<input.rows;k++)
	{
		for (auto l=0;l<input.cols;l++)
		{
			if (input.at<int>(k,l) <= 65000)
			{
				if (input.at<int>(k,l) > maximum)
				{
					maximum = input.at<int>(k,l);
				}	
			}
		}
	}

	for(auto k=0;k<input.rows;k++)
	{
		for (auto l=0;l<input.cols;l++)
		{
			if (input.at<int>(k,l) > 65000)
			{
				den_img.at<int>(k,l)=maximum;
			}else
			{
				den_img.at<int>(k,l)=input.at<int>(k,l);
			}
		}
	}

	return den_img;
	//matlab equiv
	//[m, n, v] = find(img1>65000);
	//num=size(m,1);
	//for kk=1:num
	//    img2(m(kk), n(kk))=0;
	//end
	//imax=max(max(img2));
	//for ii=1:num
	//    img1(m(ii), n(ii))=imax;
	//end
}
///////////////////////////////////////////////////////////////////////////////////
cv::Mat sr4000handler::normalizeImage(cv::Mat input)
{
	Mat norm_img = Mat::zeros(cv::Size(input.cols,input.rows), CV_8U);
	Mat trans_img = Mat::zeros(cv::Size(input.cols,input.rows), CV_64F);
	
	double minimum=10000000.0, maximum=-10000000.0;
	for(auto k=0;k<input.rows;k++)
	{
		for (auto l=0;l<input.cols;l++)
		{
			if (input.at<int>(k,l) < minimum)
			{
				minimum = input.at<int>(k,l);
			}

			if (input.at<int>(k,l) > maximum)
			{
				maximum = input.at<int>(k,l);
			}
		}
	}

	for(auto k=0;k<input.rows;k++)
	{
		for (auto l=0;l<input.cols;l++)
		{
			trans_img.at<double>(k,l)=(sqrt(double(input.at<int>(k,l)))/sqrt(maximum))*255.0;
		}
	}

	//rerun the min max calculation loops
	minimum=10000000.0, maximum=-10000000.0;
	for(auto k=0;k<input.rows;k++) 
	{
		for (auto l=0;l<input.cols;l++)
		{
			if (trans_img.at<double>(k,l) < minimum)
			{
				minimum = trans_img.at<double>(k,l);
			}

			if (trans_img.at<double>(k,l) > maximum)
			{
				maximum = trans_img.at<double>(k,l);
			}
		}
	}
	//TBD: see if this loop is required
	for(auto k=0;k<input.rows;k++)
	{
		for (auto l=0;l<input.cols;l++)
		{
			double numer=trans_img.at<double>(k,l)-minimum;
			double denom=maximum-minimum;

			trans_img.at<double>(k,l)=(numer/denom);
		}
	}
	
	//cv::normalize(input,trans_img, 0, 1, cv::NORM_MINMAX);
	trans_img.convertTo(norm_img, CV_8U, 255, 0);
	return norm_img;
	//matlab equiv
	//im = sqrt(img1) / sqrt(max(img1(:)))*255 ;
	//im = (  im - min(im(:)) ) / (  max(im(:))-min(im(:)) )*255;
}
///////////////////////////////////////////////////////////////////////////////////////
cv::Mat sr4000handler::gblurImage(cv::Mat input)
{
	Mat blur_img;// = Mat::zeros(cv::Size(input.cols,input.rows), CV_64F);
	GaussianBlur(input,blur_img, Size(5,5), 2.5, 2.5 );//param 3 is std.dev in x (2.5), param 4 is std.dev in y(2.5)
	return blur_img;
}
///////////////////////////////////////////////////////////////////////////////////////
