#include <iostream>
#include <fstream>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "square.hpp"
#include "ball.hpp"

#define BACK_COLOR {255, 255, 255} //RGB COLOR
#define SQUARE_COLOR {255, 255, 255} //RGB COLOR

#define SQUARE_CENTER {0, 0, 900}
#define SQUARE_NORMAL {0, 0, -1}  //MUST BE NORMALIZED 
#define SQUARE_V1 {-200, -200, 900} //MUST BE NORMALIZED

#define MARKER_SIZE 1.0/2

#define BALL_RADIUS 30

/**********************************************************************/
#define SET_IMAGE_BACKGROUND //ELSE USE BACKGROUND WITH BACK_COLOR
#define INSERT_SQUARE
//#define USE_COLOR_MODE //ELSE USE COLORS FROM REFERENCE IMAGE
#define PRINT_BALL_COLOR
/**********************************************************************/

int main(int argc, char* argv[])
{
	cv::Mat K = cv::Mat(3, 3, CV_32F);
	cv::Mat K_inv = cv::Mat(3, 3, CV_32F); 
	cv::Mat trajectory;
	cv::Mat background;
	cv::Mat ref_image;
	
	//GET PARAMETERS BY COMMAND ARGUMENTS
	if(argc == 5) 
	{
		//AUXILIAR STRING
		std::string s; 
		//AUXILIAR FILE OBJECT
		std::ifstream f; 
		
		//GET COLOR HISTOGRAM OF BALL MODEL 
		ref_image = cv::imread("modelFiles/" + std::string(argv[1]));
		
		//GET SIZE TO GENERATE IMAGE
		s = argv[4];
		uint im_size[2]; //width height
		
		im_size[0] = std::stoi(s.substr(0, s.find("x")));
		im_size[1] = std::stoi(s.substr(s.find("x") + 1));
		background = cv::Mat(im_size[1], im_size[0], CV_8UC3);
		
		//GET K MATRIX
		s = "camMatrices/" + std::string(argv[3]);

		f.open(s);
		getline(f, s, ' ');  K.at<float>(0, 0) = std::stof(s);
		getline(f, s, ' ');  K.at<float>(0, 1) = std::stof(s);
		getline(f, s, '\n'); K.at<float>(0, 2) = std::stof(s);
		getline(f, s, ' ');  K.at<float>(1, 0) = std::stof(s);
		getline(f, s, ' ');  K.at<float>(1, 1) = std::stof(s);
		getline(f, s, '\n'); K.at<float>(1, 2) = std::stof(s);
		getline(f, s, ' ');  K.at<float>(2, 0) = std::stof(s);
		getline(f, s, ' ');  K.at<float>(2, 1) = std::stof(s);
		getline(f, s, '\n'); K.at<float>(2, 2) = std::stof(s);
		f.close();
		
		//GET BALL TRAJECTORY
		std::vector<float> v;
		std::string trajectory_file;
		trajectory_file = "trajectories/" + std::string(argv[2]);

		f.open(trajectory_file);
		while(true)
		{	
			getline(f, s);
			if(s.empty())
				break;

			uint space1 = s.find(" ");
			uint space2 = space1 + s.substr(s.find(" ") + 1).find(" ");
			
			v.push_back(std::stof(s.substr(0, space1)));
			v.push_back(std::stof(s.substr(space1 + 1, space2)));
			v.push_back(std::stof(s.substr(space2 + 1)));
		}
		f.close();
		
		uint n_pts = v.size()/3;
		trajectory = cv::Mat(n_pts, 3, CV_32F);
		for(uint i = 0; i < n_pts; i++)
		{
			trajectory.at<float>(i , 0) = v[3*i];
			trajectory.at<float>(i , 1) = v[3*i + 1];
			trajectory.at<float>(i , 2) = v[3*i + 2];
		}	
	}
	else
	{
		std::cout << "Wrong parameters\n";
		return 1;
	}

	//SET BALL PARAMETERS	
	Ball ball(BALL_RADIUS, ref_image, K);
#ifdef USE_COLOR_MODE
	ball.useColorMode(true);
#endif

	//SET SQUARE PARAMETERS
	float sc[3] = SQUARE_CENTER;
	float sn[3] = SQUARE_NORMAL;
	float v1[3] = SQUARE_V1;
	Square square(sc, sn, v1, (float)MARKER_SIZE, SQUARE_COLOR, K);
	
#ifdef PRINT_BALL_COLOR
	std::cout << "BALL COLOR: " << ball.colorMode() << '\n';
#endif

	//SET BACKGROUND
#ifdef SET_IMAGE_BACKGROUND
	background = cv::imread("modelFiles/background.png");
#else
	for(uint j = 0; j < (uint)background.rows; j++)
		for(uint i = 0; i < (uint)background.cols; i++)
		{	
			uint8_t backAux[] = BACK_COLOR;
			background.at<cv::Vec3b>(cv::Point(i, j)) = {backAux[2], 
														 backAux[1], 
														 backAux[0]};
		}
#endif

#ifdef INSERT_SQUARE		
	square.insertSquare(background);
#endif

	std::string traj = std::string(argv[2]);
	auto idx = traj.find(".");
	traj = traj.substr(0, idx);

	for(uint k = 0; k < (uint)trajectory.rows; k++)
	{
		cv::Mat image = background.clone();
		ball.insertBall(image, trajectory.row(k));
			
		//WRITE PNG IMAGE
		cv::imwrite("output/" + traj + "/im_" + 
								std::to_string(k + 1) + ".png", image);
		std::cout << "IMAGE: " << k << " | BALL CENTER: " 
				  << trajectory.row(k) << '\n';
	}
}
