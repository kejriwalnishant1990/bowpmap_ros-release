/*
  Date 24 November, 2014
  This program is written for topological mapping using kdtree, visual word pair observation in
  the likelihood. This is extension of the program augTopoMapWordPairCC.cpp.
  This program is being written for running any general dataset for topological mapping.
  It will include time complexity analysis.

  Date 25 November, 2014
  This module is incorporated with timing analysis of different module in the graph.

  Date 11 December, 2014
  This code is developed with incremental approach for building visual dictionary. We are using
  k-d tree for indexing the words to get efficient search time.

  Date 24 December , 2014
  There is a problem with incremental dictionary that we cant differentaite words repeatability for the iteratores
  where we are not building the tree(same is for word-pair). As  we are just adding all words available in different images.
  However we are checking same words in the new words.(If same word is present in the newwordVector). This has to be fixed
  and is a genune question which can be raised.

  Date 29 Dec, 2014
  This code is copy of incrementalTopoMapFinal.cpp. It has been created just to create a backup of code. Now
  current working code is incrementalTopoMapFinal1.cpp.

  Date 2 Jan, 2015
  This code is copied from incrementalTopoMapFinal1.cpp. To incorporate incremental addition of dictionary
  in the k-d tree, we will use external flann library since incremental addition is not supported in opencv flann.

  Date 28Jan, 2015
  This code is final version of bowp submitted in RSS 2015. It will further extended for next submission.
  This code is implemented to find the results on the stereoimages in the new college and city center datasets.
  A new ground truth is downladed from RTaB-Map results. Left and right images are combined to find new precision and recall
  for performance comparison.

  Date 19 Feb, 2015
  This code has been cleaned with un-necesaary codes. It is a working code.

  Date 19 Feb, 2015
  Until now a geometrical verification step was performed on the maximum probability candidate.
  We will test it will K candidate geometrical verification. It should improve result on Bovisa dataset.


  Date 8 April, 2015
  We will find the dominant features for each node and keep them for a higher level node representation.
  It should help in finding a set of possible loop closure candidates and these candidate will be further
  verified with detailed representation of the image with all features. If the node having maximum matching
  satisfy geoemtrical verification step, it will be considered as loop closure.
  In common terms
    Acquire Image -> Reach to neighborhood -> Reach to specific node -> Verify and take decision
                   (Using HIERARCHICAL_NODES) (Using tf-idf concept)    (Using RANSAC)

  Date 16 April, 2015
  Hierarchical clustering is not implemented in this code. Just a segment of it. Keep its flag 0 only.


  Date 23 April, 2015
  Adding Locality Sensitive hashing as an alternate to K-D tree.

  */

#include <iostream>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/ml/ml.hpp>
#include <stdio.h>
#include <fstream>
#include <bowpmap_ros/surflib.h>

#include <bowpmap_ros/incrementalTopoMapFinal.h>
#include <bowpmap_ros/gnuplot_ci.h>
#include <sys/time.h>
#include <flann/flann.hpp>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

//#define DIST_RATIO                  0.3        // Distance ratio threshold for kdtree nearest enighbors
#define INITIAL_HYPOTHESIS          5           // Number of initial hypothesis to be created before considering any loop closure
//#define LOOP_CLOSURE_HYPOTHESIS_THRESHOLD 0.1   // Thresold for loop closure probability
#define GNUPLOTS                    1           // Set 1 for plotting the results on gnuplot
#define WAITKEY                     0           // Set 1 for enabling wait at the end of every frame
#define VISUAL_PAIRS                1           // Set 1 for results with word-pair dictionary
#define TEST_PHASE                  0           // Set 1 for running modules in the test phase, 0 otherwise
#define COMPUTE_TIME                0           // Set 1 for computing time complexity
#define ONLINE_DICTIONARY           1           // Set 1 for using incremental online generation
//#define HESSAIN_THRESHOLD           100         // Set hessain threshold for selection of number of features.
#define EFFICIENT_WORD_PAIR_SEARCH  1       // Set 1 for efficient word-pair search
#define EXTERNAL_FLANN_LIBRARY      1          // Set 1 for using external FLANN library
#define OPENCV_COMPUTE_TIME         1           //set 1 for using opencv library for computing execution time
//#define DESCRIPTORS_PER_IMAGE       1000         // Set the number of descriptors per image having maximum dominance
//#define NO_OF_VERIFICATION_CANDIDATES   1       // Number of candidates selected for geometrical verification
#define HIERARCHICAL_NODES          0           // Set 1 for enabling hierarchical clustering
#define KDTREE                      1           // Set 1 for kd tree based indexing
#define LSH                         0           // Set 1 for LSh based indexing
#define SHOW_LOOP_CLOSURE_IMAGE     1
#define OPENCV_FEATURE_DETECTOR     0

#define WXT_TERMINAL_DISP           1           //Set 1 to see output in wxt terminal.
#define ANIMATION_DISP              0           //Set 1 to store output as gif file.

#if(!OPENCV_COMPUTE_TIME)
#include <ros/time.h>
#endif



using namespace std;



class BowpMap
{
protected :
private:

public:

    /*
     * Variable parameters
     */
    ofstream single_file_plot;

    double DIST_RATIO;
//    int INITIAL_HYPOTHESIS;
    double LOOP_CLOSURE_HYPOTHESIS_THRESHOLD;
    int HESSAIN_THRESHOLD;
    int DESCRIPTORS_PER_IMAGE;
    int NO_OF_VERIFICATION_CANDIDATES;
    double MATCHING_THRESHOLD;
    int node_i,node_j;

//  Decleration of variables for odometryCallBack function.

    double initial_pos_a,initial_pos_b, yaw, initial_yaw;
    int image_capture_flag,update_initial_pos_flag;

//  Decleration of vector to store odometry reading for new nodes.

//    vector <double> odom_reading_for_new_nodes_x;
//    vector <double> odom_reading_for_new_nodes_y;
    vector<cv::Point2f> odom_reading_for_new_nodes_xy;
    vector<int> loop_closure_node_number;

//    string image_topic;


//    recent






    //*********************************************************************

    ros::NodeHandle nImage;

    bool loopClosureDecision;

    ros::Publisher loopClosureEventPublisher;

    ros::Publisher loopClosureImagePublisher;

    vector<cv::Mat> vectorOfNewWordsForFLANN;
    char datasetFolder[200];

    string descriptorFileName;
    ofstream descriptorFile;

#if(SHOW_LOOP_CLOSURE_IMAGE)
    vector<cv::Mat> nodeImages;
#endif

#if(COMPUTE_TIME || OPENCV_COMPUTE_TIME)
    string filename;
    ofstream timeFile;
    double avgsurfDetectDiffTime,avgdescriptorSearchDifftime;
    double avgtransitionDiffTime,avgloopDiffTime;
    double avgmatchScoreDiffTime;
    double avgransacDiffTime;
    double avgWordPairSearchDiffTime;
    double avgNodeUpdateTime ;
    double avgTreeBuildTime ;
    double avgwordTableUpdateTime ,avgWordPairTableUpdateTime ;
    double avgIdfCalculationTime ;

#endif

    double avgDescriptors ,currentDescriptors;

#if(TEST_PHASE && VISUAL_PAIRS)
    // File for writing kldivergence score
    string klFileName;
    ofstream klFile;

    string loopStatusFileName;
    ofstream loopStatusFile;

    string newNodeStatusFileName;
    ofstream newNodeStatus;


#endif

#if(OPENCV_FEATURE_DETECTOR)
    cv::Ptr<cv::FeatureDetector> featureDetector;
    cv::Ptr<cv::DescriptorExtractor> descriptorExtractor;
//    cv::SurfFeatureDetector featureDetector;
//    cv::SurfDescriptorExtractor descriptorExtractor;
//    cv::SURF surf;

#endif

    // Final output result file
    string finalResultFileName;
    ofstream finalResults;

    ////////////////////////////////////////////////////////////////////////////////////////
    //Nodes order in with images
    vector<int> nodeSequenceInTime;
    ////////////////////////////////////////////////////////////////////////////////////////



    // Vector storing idf values corresponding to each word
    vector<double> idfValueVector;

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Section for incremental dictionary generation
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    cv::Mat featureDataset;
    cv::Mat tempFeatureDataset;


    string loopClosureFileName;
    ofstream loopClosureFile;


#if(KDTREE)

#if(EXTERNAL_FLANN_LIBRARY)
//    flann::KDTreeIndexParams KDparam;
//    flann::Index< flann::L2<float> > index;
#else
    cv::flann::KDTreeIndexParams KDparam;
    cv::flann::Index index;
#endif

#elif(LSH)


#endif


    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Inverted Index table
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    vector< vector<int> > wordNodeIndexTable;
    vector< vector<int> > tempWordNodeIndexTable;

#if(VISUAL_PAIRS)
    vector< vector<int> > wordPairNodeIndexTable;
    vector< vector<int> > tempWordPairNodeIndexTable;
    vector<double> idfPairValueVector;

    vector<int> countOfPairsCorrespodingToEachWord;

#if(EFFICIENT_WORD_PAIR_SEARCH)
    multimap <int, vector<int> > mytempWordPairsDictionaryTable;
    multimap< int,vector<int> > myWordPairsDictionaryTable;
#else
    vector<pair<int,int> > wordPairsDictionaryTable;
    vector<pair<int,int> > tempWordPairsDictionaryTable;
#endif

#endif



    // Variable storing the neighbor of each node ( 4 back and 4 front)
    map<int,vector<int> > neighborInfo;


    // Hierarchical clustering parameters

#if(HIERARCHICAL_NODES)
    vector< vector<int> > dominantWordNodeIndexTable;
#endif

    //***************************************************************************************************
    // Number of partitions
    int partitionCount ;

    // Number of loop closures
    int loopClosureCount;

    // Number of nodes
    int node_number;

    // Last robot position Node
    int lastRobotPositionNode ;

    // Array containing each partition details
    vector< vector<int> > partitionDetails;

    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Bayes Filter parameters
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    int numOfHypothesis ;

    vector<double> priorVector;
    vector<double> posteriorVector;

    // Frame Counts (Just a Count)
    int frameNumber ;

    //*************************************************************************************************
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Array containing each individual node Descriptors and Keypoints
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    vector< vector<cv::KeyPoint> > nodeKeyPointVector;
    vector< cv::Mat> nodeDescriptorVector;
    vector<int> nodeImageIndex;
    //    vector< vector<double> > nodeHistogramVector;




    //******************************************************************************************************
    //******************************************************************************************************
    // Main Loop starts here

    //******************************************************************************************************

    vector<int> wordsPresentInCurrentImage;
    cv::Mat newDescriptorInCurrentImage;
    vector<int> newWordsNotInDictionaryInCurrentImage;
    vector<cv::Point2f> newWordsLocationInImage;
    vector<int> newWordsRadiusInImage;

#if(EXTERNAL_FLANN_LIBRARY)
    flann::Matrix<float> flannFeatureDataset;
#endif

    flann::Index< flann::L2<float> > *index1;

#if(GNUPLOTS)

    gnuplot_ci::GP_handle *G2;
    gnuplot_ci::GP_handle *G_map;
    gnuplot_ci::GP_handle *G1;

#endif


    void loopClosureCallBack(const sensor_msgs::ImageConstPtr &image);
    void odometryCallback(const nav_msgs::Odometry::ConstPtr &odometry);
    void identifyPlaceForCameraImage(cv::Mat &cameraImage, cv::Mat &lcdResultImage);
    flann::Index< flann::L2<float> > initKDtree();


    // Class Constructor

    BowpMap()
    {
        node_i=node_j=0;

        NO_OF_VERIFICATION_CANDIDATES = 1;
        nImage.param("DIST_RATIO",DIST_RATIO,0.5);
//        nImage.param("INITIAL_HYPOTHESIS",INITIAL_HYPOTHESIS,5);
        nImage.param("MATCHING_THRESHOLD",MATCHING_THRESHOLD,4.0);
        nImage.param("HESSAIN_THRESHOLD",HESSAIN_THRESHOLD,100);
        nImage.param("DESCRIPTORS_PER_IMAGE",DESCRIPTORS_PER_IMAGE,1000);
        nImage.param("NO_OF_VERIFICATION_CANDIDATES",NO_OF_VERIFICATION_CANDIDATES,1);
        nImage.param("LOOP_CLOSURE_HYPOTHESIS_THRESHOLD",LOOP_CLOSURE_HYPOTHESIS_THRESHOLD,0.1);
//        nImage.param<std::string>("image_topic",image_topic,"/camera/rgb/image_color");

//      Definition of variables for odometryCallBack function.
        initial_pos_a=0,initial_pos_b=0;
        image_capture_flag=0,update_initial_pos_flag=0;

    #if(COMPUTE_TIME)
    ros::Time::init();
    #endif

    // Removing all previously stored images in the Result Folder
    system("rm -r ./Results");

    // Creating a folder for storing results for each query image
    system("mkdir -p ./Results/");

    // Removing all previously stored images in the QueryImage Folder
    system("rm -r ./QueryImage");

    // Creating a folder for storing results for each query image
    system("mkdir -p ./QueryImage/");

    loopClosureDecision = false;


    descriptorFileName.append(datasetFolder);
    descriptorFileName.append("descriptorFile.txt");
    descriptorFile.open(descriptorFileName.c_str());

    //*************************************************************************************************************
#if(COMPUTE_TIME || OPENCV_COMPUTE_TIME)
    // Timing analysis file of different components
    filename.append(datasetFolder);
    filename.append("timeFile.txt");
    timeFile.open(filename.c_str());


     avgsurfDetectDiffTime = 0.0,avgdescriptorSearchDifftime = 0.0;
     avgtransitionDiffTime = 0.0,avgloopDiffTime = 0.0;
     avgmatchScoreDiffTime = 0.0;
     avgransacDiffTime = 0.0;
     avgWordPairSearchDiffTime = 0.0;
     avgNodeUpdateTime = 0.0;
     avgTreeBuildTime = 0.0;
     avgwordTableUpdateTime = 0.0,avgWordPairTableUpdateTime = 0.0;
     avgIdfCalculationTime = 0.0;


#endif


     avgDescriptors = 0.0, currentDescriptors = 0.0;


    //********************************************************************************************
#if(TEST_PHASE && VISUAL_PAIRS)


    klFileName.append(datasetFolder);
    klFileName.append("klFilewordPair.txt");
    klFile.open(klFileName.c_str());

    loopStatusFileName.append(datasetFolder);
    loopStatusFileName.append("loopStatusFilewordPair.txt");
    loopStatusFile.open(loopStatusFileName.c_str());

    newNodeStatusFileName.append(datasetFolder);
    newNodeStatusFileName.append("newNodeStatusFile.txt");
    newNodeStatus.open(newNodeStatusFileName.c_str());

#endif



    //********************************************************************************************************


#if(GNUPLOTS)

      G2 = new gnuplot_ci::GP_handle("/usr/bin/", "X", "Y", "Z");
//    G_map = new gnuplot_ci::GP_handle("/usr/bin/", "X", "Y", "Z");
//    G1 = new gnuplot_ci::GP_handle("/usr/bin/", "X", "Y", "Z");
//    gnuplot_ci::GP_handle G1("/usr/bin/", "X", "Y", "Z");
//    gnuplot_ci::GP_handle G2("/usr/bin/", "X", "Y", "Z");
//    gnuplot_ci::GP_handle G3("/usr/bin/", "X", "Y", "Z");
//    gnuplot_ci::GP_handle G4("/usr/bin/", "X", "Y", "Z");
//    gnuplot_ci::GP_handle G5("/usr/bin/", "X", "Y", "Z");
//    gnuplot_ci::GP_handle G_map("/usr/bin/", "X", "Y", "Z");

#if(WXT_TERMINAL_DISP)
      G2->gnuplot_cmd("set terminal wxt size 1300,600");
#endif
//      G1->gnuplot_cmd("set terminal gif size 1300,600 animate delay 50");
//      G1->gnuplot_cmd("set output 'animation.gif'");

#if(ANIMATION_DISP)
      G2->gnuplot_cmd("set terminal gif size 1300,600 animate delay 50");
      G2->gnuplot_cmd("set output 'topo-map.gif'");
#endif

    //    G_image.gnuplot_cmd("set multiplot layout 2,1");
    //    G_image.gnuplot_cmd("unset key");
    //    G_image.gnuplot_cmd("unset xtics");
    //    G_image.gnuplot_cmd("unset ytics");
    //    G_image.gnuplot_cmd("set size ratio -1");


    //    G3.gnuplot_cmd("set terminal gif animate delay 50");
    //    G3.gnuplot_cmd("set outrput 'likelihood.gif'");

    //    G1.gnuplot_cmd("set terminal gif animate delay 50");
    //    G1.gnuplot_cmd("set output 'prior.gif'");

    //    G5.gnuplot_cmd("set terminal gif animate delay 50");
    //    G5.gnuplot_cmd("set output 'predicted.gif'");
#endif
    //*****************************************************************************************************

#if(OPENCV_FEATURE_DETECTOR)
    cv::initModule_nonfree();

    featureDetector = cv::FeatureDetector::create("SURF");

    descriptorExtractor = cv::DescriptorExtractor::create("SURF");

//    featureDetector->set("hessianThreshold",int(HESSAIN_THRESHOLD));
//    featureDetector->setInt("extended",0);
//    descriptorExtractor->setInt("extended",0);
//    descriptorExtractor->setInt("hessianThreshold",int(HESSAIN_THRESHOLD));


//    surf = cv::SURF(double(HESSAIN_THRESHOLD));
#else
#endif

    //**********************************************************************************************

    // Final result file of different components
    finalResultFileName.append(datasetFolder);
    finalResultFileName.append("finalResults.txt");
    finalResults.open(finalResultFileName.c_str());
    if(!finalResults.is_open())
    {
        cerr << "Error opening finalResults.txt" << endl;
        exit(0);
    }




    loopClosureFileName.append(datasetFolder);
    loopClosureFileName.append("loopClosureFile.txt");
    loopClosureFile.open(loopClosureFileName.c_str());
    if(!loopClosureFile.is_open())
    {
        cerr << "Error opening loopClosureFile.txt" << endl;
        exit(0);
    }





#if(KDTREE)

#if(EXTERNAL_FLANN_LIBRARY)
//    KDparam = flann::KDTreeIndexParams(1);

//    flann::Index< flann::L2<float> > index(flann::KDTreeIndexParams(1));
    index1 = new flann::Index< flann::L2<float> > (flann::KDTreeIndexParams(1));

#else
     KDparam = cv::flann::KDTreeIndexParams(1);
#endif

#elif(LSH)


#endif

    node_i=0;
    node_j=0;



   //***************************************************************************************************
    // Number of partitions
    partitionCount = 0;

    // Number of loop closures
    loopClosureCount = 0;

    // Number of nodes
    node_number = 0;

    // Last robot position Node
    lastRobotPositionNode = -1;


    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Bayes Filter parameters
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    numOfHypothesis = 0;

    // Frame Counts (Just a Count)
    frameNumber = 0;

    }

    // Class destructor

    ~BowpMap()
    {

        //    double recall = double(truePositiveLoopClosures)/int(GROUND_TRUTH);
        //    double precision = double(truePositiveLoopClosures)/(truePositiveLoopClosures+actualFalsePositives);
        //    double fscore = 2*double(precision*recall)/(precision+recall);

        //    finalResults << "True positive loop closures \t" << truePositiveLoopClosures << endl;
        //    finalResults << "False positives loop closures \t" << falsePositiveLoopClosures << endl;
        //    finalResults << "Loop closures in the same loop which will be counted in false positives \t" << loopClosuresInTheSameLoop << endl;
        //    finalResults << "Actual false positives excluding loop closures in the same loop \t" << actualFalsePositives << endl;
            finalResults << "total Number of nodes \t" << partitionCount << endl;
        //    finalResults << "Precision \t" << precision << endl;
        //    finalResults << "Recall \t" << recall << endl;
        //    finalResults << "fscore \t" << fscore << endl;
            finalResults << "Matching Threshold \t" << double(MATCHING_THRESHOLD) << endl;
            finalResults << "Hessian Threshold \t" << int(HESSAIN_THRESHOLD) << endl;
        #if(VISUAL_PAIRS)
            finalResults << "Size of the word-pair dictionary \t" << wordPairNodeIndexTable.size() << "\t" << myWordPairsDictionaryTable.size() <<
                            "\t" << mytempWordPairsDictionaryTable.size() << endl;
        #endif
            finalResults << "Size of the dictionary \t" << wordNodeIndexTable.size() << "\t" << featureDataset.rows << endl;
            finalResults << "Average computation time per image \t" << avgloopDiffTime << endl;
            finalResults << "tree building time per image  \t" << avgTreeBuildTime << endl;
            finalResults << "tree search time per image \t" << avgdescriptorSearchDifftime << endl;
            finalResults << "word -pair search time \t" << avgWordPairSearchDiffTime << endl;
            finalResults << "Average descriptors per image \t" << avgDescriptors << endl;
            finalResults << "Loop Closure Probability \t" << double(LOOP_CLOSURE_HYPOTHESIS_THRESHOLD) << endl;
            finalResults << "DIST ratio \t" << double(DIST_RATIO) << endl;


            cout << "Number of loop closures \t" << loopClosureCount << endl;
    descriptorFile.close();
    finalResults.close();

#if(COMPUTE_TIME || OPENCV_COMPUTE_TIME)
    timeFile.close();
#endif
#if(TEST_PHASE)
    klFile.close();
    loopStatusFile.close();
#endif
    }


};

flann::Index< flann::L2<float> > BowpMap::initKDtree()
{
    flann::Index< flann::L2<float> > index(flann::KDTreeIndexParams(1));
    return index;
}

void BowpMap::identifyPlaceForCameraImage(cv::Mat &dstImg,cv::Mat &lcdResultImage)
{

        int static imgCount = 0;
        frameNumber++;


        double surfDetectDiffTime = 0.0,descriptorSearchDifftime = 0.0;
        double transitionDiffTime = 0.0,loopDiffTime = 0.0;
        double matchScoreDiffTime = 0.0;
        double ransacDiffTime = 0.0;
        double wordPairSearchDiffTime = 0.0;
        double nodeUpdateDiffTime = 0.0;
        double idfValueDiffTime = 0.0, wordTableUpdateDiffTime = 0.0, wordPairTableUpdateDiffTime = 0.0;
        double treeBuildDiffTime = 0.0;

#if(COMPUTE_TIME)
        ros::Time surfDetectStartTime,surfDetectEndTime;
        ros::Time descriptorSearchStartTime, descriptorSearchEndTime;
        ros::Time transitionCalcStartTime,transitionCalcEndTime;
        ros::Time loopStartTime,loopEndTime;
        ros::Time matchScoreStartTime, matchScoreEndTime;
        ros::Time ransacStartTime , ransacEndTime ;
        ros::Time wordPairSearchStartTime, wordPairSearchEndtime ;
        ros::Time nodeUpdateStartTime, nodeUpdateEndTime;
        loopStartTime = ros::Time::now();
#elif(OPENCV_COMPUTE_TIME)
        double surfDetectStartTime = 0.0,surfDetectEndTime = 0.0,descriptorSearchStartTime = 0.0, descriptorSearchEndTime = 0.0,
                transitionCalcStartTime=0.0,transitionCalcEndTime= 0.0,loopStartTime = 0.0,loopEndTime =0.0,matchScoreStartTime = 0.0, matchScoreEndTime = 0.0,
                ransacStartTime=0.0,ransacEndTime=0.0,wordPairSearchStartTime=0.0,wordPairSearchEndtime=0.0,
                nodeUpdateStartTime=0.0,nodeUpdateEndTime=0.0;
        loopStartTime = (double)cv::getTickCount();
#endif

        int loopClosureCandidate = 0;
        double loopClosureProbability = 0.0;

        vector<cv::KeyPoint> dstKeypoint;
        cv::Mat dstDescriptors;

#if(COMPUTE_TIME)
        surfDetectStartTime = ros::Time::now();
#elif(OPENCV_COMPUTE_TIME)
        surfDetectStartTime = (double)cv::getTickCount();
#endif

        vector<cv::KeyPoint> testKeyPoint;
        cv::Mat testDesc;

//        cv::Mat dstImg = cv::imread(imgFileNames[imgCount]);

#if(OPENCV_FEATURE_DETECTOR)
        featureDetector->detect(dstImg,testKeyPoint);

        descriptorExtractor->compute(dstImg,testKeyPoint,testDesc);

#else
               IplImage *img1 = new IplImage(dstImg);

               IpVec ipts1;
               surfDetDes(img1,ipts1,false,2,2,2,0.0);


               for(int j=0;j<ipts1.size();j++)
               {
                   cv::Mat tempMat = cv::Mat(1,64,CV_32FC1,0.0);
                   cv::KeyPoint tempKey;
                   for(int k=0;k<64;k++)
                   {
                       tempMat.at<float>(0,k) = ipts1[j].descriptor[k];
                       tempKey.angle = ipts1[j].orientation;
                       tempKey.pt.x = ipts1[j].x;
                       tempKey.pt.y = ipts1[j].y;

                   }
                   testDesc.push_back(tempMat);
                   testKeyPoint.push_back(tempKey);
                   tempMat.release();

               }

#endif



        findFixedFeatures(testKeyPoint,dstKeypoint,testDesc,dstDescriptors,DESCRIPTORS_PER_IMAGE);



        cout << "Image number ********************************************* \t" << imgCount << "\t" <<
                dstDescriptors.rows << "\t" << dstDescriptors.cols << endl;

        currentDescriptors = dstDescriptors.rows;
        avgDescriptors = (double)(((frameNumber-1) * avgDescriptors) + currentDescriptors)/(frameNumber);

        descriptorFile << imgCount+1 << "\t" << currentDescriptors << "\t" << avgDescriptors << endl;



#if(COMPUTE_TIME)
        surfDetectEndTime = ros::Time::now();
#elif(OPENCV_COMPUTE_TIME)
        surfDetectEndTime = (double)cv::getTickCount();
#endif

        vector<int> wordFrequencyInCurrentImage(wordNodeIndexTable.size(),0);
        vector<double> currentImgHist(wordNodeIndexTable.size(),0.0);



        int totalWordsInCurrentImage = 0;

#if(VISUAL_PAIRS)
        vector<int> wordList;
        vector<cv::Point2f> wordLocationInImage;
        vector<int> wordPairVector(wordPairNodeIndexTable.size(),0);
        vector<double> currentImgWordPairHist(wordPairNodeIndexTable.size(),0.0);
        int totalVisualWordPairInCurrentImage = 0;
        vector<double> wordRadius;
#endif

#if(COMPUTE_TIME)
        descriptorSearchStartTime = ros::Time::now();
#elif(OPENCV_COMPUTE_TIME)
        descriptorSearchStartTime = (double)cv::getTickCount();
#endif

        for(int descCount = 0; descCount < dstDescriptors.rows ; descCount++)
        {
            int knn = 2;

#if(EXTERNAL_FLANN_LIBRARY)
            flann::Matrix<float> query;
            query = flann::Matrix<float>((float*) dstDescriptors.row(descCount).data,1,dstDescriptors.row(descCount).cols);
            flann::Matrix<int> indices(new int[1*knn],1,knn);
            flann::Matrix<float> dists(new float[1*knn],1,knn);
            flann::SearchParams params = flann::SearchParams(128);
#else
            cv::Mat indices,dists;
            const cv::flann::SearchParams& params=cv::flann::SearchParams();
#endif

            float dist_ratio = 10 ;
            //            cout << "DescCount \t" << descCount << "\t" << query.cols << "\t" << query.rows << "\t"
            //                     << index.veclen() << endl;


            if(imgCount > 0)
            {
#if(EXTERNAL_FLANN_LIBRARY)

                index1->knnSearch(query,indices,dists,knn,params);
                dist_ratio = float(dists[0][0]/dists[0][1]);
                //            cout << dist_ratio << endl;
#else

                index.knnSearch(dstDescriptors.row(descCount),indices,dists,knn,params);

                // Calculating distance ratio to decide whether kdtree result satisy distance ratio(probable candidate)
                dist_ratio = float(dists.at<float>(0)/dists.at<float>(1));
#endif

            }


            if(dist_ratio < DIST_RATIO)
            {
                totalWordsInCurrentImage++;

#if(EXTERNAL_FLANN_LIBRARY)
                wordFrequencyInCurrentImage[indices[0][0]]++;
                wordsPresentInCurrentImage.push_back(indices[0][0]);
#else
                // This is a good match according to a dist_ratio criteria.
                wordFrequencyInCurrentImage[indices.at<int>(0)]++;

                wordsPresentInCurrentImage.push_back(indices.at<int>(0));

#endif
                //                cout << "Words i--n the image \t" << indices[0][0] << "\t" ;


#if(VISUAL_PAIRS)

#if(EXTERNAL_FLANN_LIBRARY)
                wordList.push_back(indices[0][0]);
#else

                wordList.push_back(indices.at<int>(0));
#endif
                wordLocationInImage.push_back(cv::Point2f(dstKeypoint[descCount].pt.x,dstKeypoint[descCount].pt.y));
                wordRadius.push_back(dstKeypoint[descCount].size/2);
#endif
            }
            else
            {

                newDescriptorInCurrentImage.push_back(dstDescriptors.row(descCount));
#if(VISUAL_PAIRS)
                newWordsNotInDictionaryInCurrentImage.push_back(featureDataset.rows+newWordsNotInDictionaryInCurrentImage.size());
                newWordsLocationInImage.push_back(cv::Point2f(dstKeypoint[descCount].pt.x,dstKeypoint[descCount].pt.y));
                newWordsRadiusInImage.push_back(dstKeypoint[descCount].size/2);
#endif

                // Not a good match
            }
            //#if(EXTERNAL_FLANN_LIBRARY)
            //            delete[] query.ptr();
            //            delete[] dists.ptr();
            //            delete[] indices.ptr();
            //#endif

        }
        cout << endl;



        // Couting number of different types of visual words present in the current image
        //        int countOfDiffWordsInCurrentImage = count_if(wordFrequencyInCurrentImage.begin(),wordFrequencyInCurrentImage.end(),IsZero);

        // Calculate bag of words representation using tf-idf scoring scheme.
        for(int i=0;i<wordNodeIndexTable.size();i++)
        {
            currentImgHist[i] = double(wordFrequencyInCurrentImage[i] *idfValueVector[i])/(totalWordsInCurrentImage);
        }



        // Normalizing the histogram generated using tf-idf score
        normalizeHist(currentImgHist,1.0);

        cout << "Number of words in the currentImage \t" << totalWordsInCurrentImage << endl;
        //        cout << "Number of different words in the image \t" << countOfDiffWordsInCurrentImage << endl;

#if(COMPUTE_TIME)
        descriptorSearchEndTime = ros::Time::now();
#elif(OPENCV_COMPUTE_TIME)
        descriptorSearchEndTime = (double)cv::getTickCount();
#endif

#if(COMPUTE_TIME)
        wordPairSearchStartTime = ros::Time::now();
#elif(OPENCV_COMPUTE_TIME)
        wordPairSearchStartTime = (double)cv::getTickCount();
#endif



#if(VISUAL_PAIRS)
        //        int totalVisualWordPairInCurrentImage = 0;
        //        std::multimap<int,int> wordPairsInCurrentImage;
        //        std::multimap<int,int> newWordPairInCurrentImage;
        vector< pair<int,int> > wordPairsInCurrentImage;
        vector< pair<int,int> > newWordPairInCurrentImage;

        vector<int> pairsPositionsFoundInwordPairs;
        double onlyPairSearchTime = 0.0;

#if(COMPUTE_TIME)
        ros::Time wpSearchStartTime = ros::Time::now();
#elif(OPENCV_COMPUTE_TIME)
        double wpSearchStartTime = (double)cv::getTickCount();
#endif


        // This dictionary we are not including word with itself
        for(int i=0;i<wordList.size();i++)
        {
            vector<int> result;
            int nN = 2;
            findWordsInARange(i,wordLocationInImage,result,nN,wordRadius[i]);

            for(int j=0;j<result.size();j++)
            {

                std::pair<int,int> testPair = std::make_pair(wordList[i],wordList[result[j]]);
                int pairPosition = -1;
#if(EFFICIENT_WORD_PAIR_SEARCH)
                findPairPositionInMultimap(myWordPairsDictionaryTable,testPair,pairPosition);
#else
                findPairInPairVector(wordPairsDictionaryTable,testPair,pairPosition);
#endif

                if(pairPosition >= 0)
                {
                    //                    cout << "pair position \t" << pairPosition << endl;
                    wordPairVector[pairPosition] += 1;
                    wordPairsInCurrentImage.push_back(testPair);
                    totalVisualWordPairInCurrentImage++;
                    pairsPositionsFoundInwordPairs.push_back(pairPosition);
                }
                else
                {
                    newWordPairInCurrentImage.push_back(testPair);
                }

            }
        }

        for(int i=0;i<newWordsNotInDictionaryInCurrentImage.size();i++)
        {
            vector<int> result;
            int nN = 2;
            findWordsInARange(i,newWordsLocationInImage,result,nN,newWordsRadiusInImage[i]);
            for(int j=0;j<result.size();j++)
            {

                std::pair<int,int> testPair = std::make_pair(newWordsNotInDictionaryInCurrentImage[i],newWordsNotInDictionaryInCurrentImage[result[j]]);
                newWordPairInCurrentImage.push_back(testPair);
            }

        }
#if(COMPUTE_TIME)
        ros::Time wpSearchEndTime = ros::Time::now();
        onlyPairSearchTime+=(wpSearchEndTime-wpSearchStartTime).toSec();
#elif(OPENCV_COMPUTE_TIME)
        double wpSearchEndTime = (double)cv::getTickCount();
        onlyPairSearchTime += (wpSearchEndTime - wpSearchStartTime)/cv::getTickFrequency();
#endif

#endif




#if(VISUAL_PAIRS)
        // start of visual pairs

#if(COMPUTE_TIME)
        ros::Time wpMakeHistStartTime = ros::Time::now();
#elif(OPENCV_COMPUTE_TIME)
        double wpMakeHistStartTime = (double)cv::getTickCount();
#endif

        double sumWordPairHist = 0.0;
        for(int i=0;i<wordPairNodeIndexTable.size();i++)
        {
            currentImgWordPairHist[i] = double(wordPairVector[i] /** idfPairValueVector[i]*/)/(totalVisualWordPairInCurrentImage);
            sumWordPairHist += currentImgWordPairHist[i];
        }

        for(int i=0;i<wordPairNodeIndexTable.size();i++)
        {
            currentImgWordPairHist[i]/=sumWordPairHist;
        }

        //        normalizeHist(currentImgWordPairHist,1.0);
        cout << "total word pairs \t" << totalVisualWordPairInCurrentImage << endl;

#if(COMPUTE_TIME)
        ros::Time wpMakeHistEndTime = ros::Time::now();
#elif(OPENCV_COMPUTE_TIME)
        double wpMakeHistEndTime = (double)cv::getTickCount();
#endif

        // End of visual pairs
#endif

#if(COMPUTE_TIME)
        wordPairSearchEndtime = ros::Time::now();
#elif(OPENCV_COMPUTE_TIME)
        wordPairSearchEndtime = (double)cv::getTickCount();
#endif


        // Flag for new node creation
        bool createNewNodeFlag = false;

        single_file_plot.open("./single_file_plot.gp");
        single_file_plot << "set multiplot title 'BoWP-Map'" <<endl;

        // Condition of zero nodes in the map or less than 5.So create first node and update the inverted index table
        if(numOfHypothesis < int(INITIAL_HYPOTHESIS))
        {
            createNewNodeFlag = true;
        }
        else
        {
            if(numOfHypothesis == int(INITIAL_HYPOTHESIS))
            {
                priorVector.resize(int(INITIAL_HYPOTHESIS),(double(1)/int(INITIAL_HYPOTHESIS)));

                for(int i=0;i<priorVector.size();i++)
                {
                    if(i==priorVector.size()-1)
                        priorVector[i] = 1.0;
                    else
                        priorVector[i] = 0.0;
                }
            }
            else
            {
                priorVector.clear();
                priorVector.assign(posteriorVector.begin(),posteriorVector.end());
            }
            //            priorVector.clear();
            //            priorVector.resize(numOfHypothesis,(double(1)/numOfHypothesis));

            // Finding the matching score of current image with the existing nodes with the help of inverted index table
#if(!VISUAL_PAIRS && COMPUTE_TIME)
            //            gettimeofday(&matchScoreStartTime,NULL);
            matchScoreStartTime = ros::Time::now();
#elif(OPENCV_COMPUTE_TIME)
            matchScoreStartTime = (double)cv::getTickCount();
#endif

            vector<double> hypothesisScore(numOfHypothesis,0.0);
            for(int i=0;i<wordNodeIndexTable.size();i++)
            {
                if(currentImgHist[i] > 0)
                {
                    for(int j=0;j<wordNodeIndexTable[i].size();j++)
                    {
                        hypothesisScore[wordNodeIndexTable[i][j]]+=currentImgHist[i];

                    }
                }
            }

#if(!VISUAL_PAIRS && COMPUTE_TIME)
            //            gettimeofday(&matchScoreEndTime,NULL);
            matchScoreEndTime = ros::Time::now();
#elif(!VISUAL_PAIRS && OPENCV_COMPUTE_TIME)
            matchScoreEndTime = (double)cv::getTickCount();
#endif

#if(VISUAL_PAIRS)

#if(COMPUTE_TIME)
            matchScoreStartTime = ros::Time::now();
#elif(OPENCV_COMPUTE_TIME)
            matchScoreStartTime = (double)cv::getTickCount();
#endif


            vector<double> hypothesisScoreFromWordPairs(numOfHypothesis,0.0);
            for(int i=0;i<wordPairNodeIndexTable.size();i++)
            {
                if(currentImgWordPairHist[i] > 0)
                {
                    for(int j=0;j<wordPairNodeIndexTable[i].size();j++)
                    {
                        hypothesisScoreFromWordPairs[wordPairNodeIndexTable[i][j]] += currentImgWordPairHist[i];
                    }
                }

            }


#if(COMPUTE_TIME)
            matchScoreEndTime = ros::Time::now();
#elif(OPENCV_COMPUTE_TIME)
            matchScoreEndTime = (double)cv::getTickCount();
#endif

#endif

#if(COMPUTE_TIME)
            transitionCalcStartTime = ros::Time::now();
#elif(OPENCV_COMPUTE_TIME)
            transitionCalcStartTime = (double)cv::getTickCount();
#endif

#if(VISUAL_PAIRS)

            vector<double> wordpairLikelihood;

            getLikelihoodTime(hypothesisScoreFromWordPairs,wordpairLikelihood,nodeSequenceInTime);


            string wordpairLikelihoodFileName;
            wordpairLikelihoodFileName.append(datasetFolder);
            wordpairLikelihoodFileName.append("wordPairLikelihood.txt");
            ofstream wordpairLikelihoodFile;
            wordpairLikelihoodFile.open(wordpairLikelihoodFileName.c_str());

            if(!wordpairLikelihoodFile.is_open())
            {
                cerr << "Error opening wordPairLikelihood.txt" << endl;
                exit(0);
            }

            for(int i=0;i<wordpairLikelihood.size();i++)
                wordpairLikelihoodFile << i << "\t" << wordpairLikelihood[i] << endl;


            wordpairLikelihoodFile.close();


//#if(GNUPLOTS)

//            G3.gnuplot_cmd("plot './wordPairLikelihood.txt' u 1:2 w l")

//#endif
#endif
            // Adding hypothesis scores
            //            for(int i=0;i<numOfHypothesis;i++)
            //            {
            //                hypothesisScore[i] += hypothesisScoreFromWordPairs[i];
            //            }

            vector<double> likelihoodVector;
            /*
              Call function to calculate the likelihood of all the hypothesis given the current obsevation
              i.e. set of descriptors
              */

            //            getLikelihood(hypothesisScore,likelihoodVector);
            getLikelihoodTime(hypothesisScore,likelihoodVector,nodeSequenceInTime);

            //            for(int i=0;i<likelihoodVector.size();i++)
            //            {
            //                likelihoodVector[i] = likelihoodVector[i]*likelihoodVector[i];
            //            }

            string wordLikelihoodFileName;
            wordLikelihoodFileName.append(datasetFolder);
            wordLikelihoodFileName.append("wordLikelhoodFile.txt");
            ofstream wordLikelihoodFile;
            wordLikelihoodFile.open(wordLikelihoodFileName.c_str());

            if(!wordLikelihoodFile.is_open())
            {
                cerr << "Error opening wordLike" << endl;
                exit(0);
            }

            for(int i=0;i< likelihoodVector.size();i++)
                wordLikelihoodFile << i << "\t" << likelihoodVector[i] << endl;

            wordLikelihoodFile.close();

//#if(GNUPLOTS)

//            G1.gnuplot_cmd("plot './wordLikelhoodFile.txt' u 1:2 w l");


//#endif


#if(TEST_PHASE && VISUAL_PAIRS)
            vector<double> testPredictVector,testPosteriorVector;
            mygetPriorDistributionNewDashTime(priorVector,testPredictVector,lastRobotPositionNode,nodeSequenceInTime);
            getPosteriorDistributionTime(testPredictVector,likelihoodVector,testPosteriorVector,nodeSequenceInTime);
#endif


#if(VISUAL_PAIRS)

            for(int i=0;i<likelihoodVector.size();i++)
            {
                likelihoodVector[i] = likelihoodVector[i] * wordpairLikelihood[i];
            }

#endif


            /* Vector after applying transition model to positerior at previous timestamp. It will be prior for the
             current timestamp.
            */
            vector<double> predictedVector;

            mygetPriorDistributionNewDashTime(priorVector,predictedVector,lastRobotPositionNode,nodeSequenceInTime);
//            predictedProbabilityWithTransitionModel(priorVector,predictedVector,lastRobotPositionNode,nodeSequenceInTime,neighborInfo);

            /*
              Calling function to calculate the posteriror given the current observation
              */
            getPosteriorDistributionTime(predictedVector,likelihoodVector,posteriorVector,nodeSequenceInTime);

            /*
                    Extract best candidate based on the posterior distribution on the basis of
                    the node which has highest probability sum in its neigborhood like if j-2,j-1
                    j,j+1,j+2.Node whose neighborhood has highest probability sum is selected as
                    loop closure candidiate.

                */

            vector<int> candidateList;
            vector<double> candidateProbability;
if(NO_OF_VERIFICATION_CANDIDATES >= 1)
{

            getKcandidatesForVerification(posteriorVector,candidateList,candidateProbability,int(NO_OF_VERIFICATION_CANDIDATES));
}
else
{
            getLoopClosureCandidateTime(posteriorVector,loopClosureCandidate,loopClosureProbability,nodeSequenceInTime);
}


#if(COMPUTE_TIME)
            transitionCalcEndTime = ros::Time::now();
#elif(OPENCV_COMPUTE_TIME)
            transitionCalcEndTime = (double)cv::getTickCount();
#endif

#if(TEST_PHASE && VISUAL_PAIRS)

            //            double klScore1 = findKLdivergenceScore(posteriorVector,testPosteriorVector);
            double klScore1 = findKLdivergenceScore(posteriorVector,predictedVector);
            double klScore2 = findKLdivergenceScore(testPosteriorVector,testPredictVector);

            klFile << imgCount << "\t" << klScore1 << "\t" << klScore2 <<  endl;
            //            klFile << imgCount << "\t" << klScore1 << endl;
#endif

#if(GNUPLOTS)
            // Creating files for writing prior, posterior and likelihood data
            ofstream priorFile,posteriorFile,likelihoodfile,hypoScoreFile,predictedVectorFile;

            priorFile.open("./priorFile.txt");
            posteriorFile.open("./posteriorFile.txt");
            likelihoodfile.open("./likelihoodFile.txt");
            hypoScoreFile.open("./hypoScoreFile.txt");
            predictedVectorFile.open("./predictedVectorFile.txt");


            if(!single_file_plot.is_open())
            {
                cerr << "Error opening priorFile.txt" << endl;
                exit(0);
            }


            if(!priorFile.is_open())
            {
                cerr << "Error opening priorFile.txt" << endl;
                exit(0);
            }

            if(!posteriorFile.is_open())
            {
                cerr << "Error opening posteriorFile.txt" << endl;
                exit(0);
            }

            if(!likelihoodfile.is_open())
            {
                cerr << "Error opening likelihoodFile.txt" << endl;
                exit(0);
            }

            if(!hypoScoreFile.is_open())
            {
                cerr << "Error opening hypoScoreFile.txt" << endl;
                exit(0);
            }

            if(!predictedVectorFile.is_open())
            {
                cerr << "Error opening predictedVectorFile.txt" << endl;
                exit(0);
            }

            for(int i=0;i<priorVector.size();i++)
                priorFile << i << "\t" << priorVector[i] << endl;

            for(int i=0;i<posteriorVector.size();i++)
                posteriorFile << i << "\t" << posteriorVector[i] << endl;

            for(int i=0;i<likelihoodVector.size();i++)
                likelihoodfile << i << "\t" << likelihoodVector[i] << endl;

            for(int i=0;i<predictedVector.size();i++)
                predictedVectorFile << i << "\t" << predictedVector[i] << endl;

            for(int i=0;i<hypothesisScore.size();i++)
                hypoScoreFile << i << "\t" << hypothesisScore[i] << endl;

#if(VISUAL_PAIRS)
            ofstream wordPairhypoScoreFile;
            wordPairhypoScoreFile.open("./wordPairHypoScoreFile.txt");
#endif

            if(!wordPairhypoScoreFile.is_open())
            {
                cerr << "Error opening hypoScoreFile.txt" << endl;
                exit(0);
            }

            for(int i=0;i<hypothesisScoreFromWordPairs.size();i++)
                wordPairhypoScoreFile << i << "\t" << hypothesisScoreFromWordPairs[i] << endl;

            wordPairhypoScoreFile.close();

            single_file_plot << "set origin 0.0,0.6" <<endl;
            single_file_plot << "set nolabel" <<endl;
            single_file_plot << "set size 0.3,0.3" <<endl;
            single_file_plot << "set size ratio 0.4" <<endl;
            single_file_plot << "set yrange [0:1]" <<endl;
            single_file_plot << "set xrange [0:*]" <<endl;
            single_file_plot << "set xlabel ''" <<endl;
            single_file_plot << "set ylabel 'Posterior'" <<endl;
            single_file_plot << "set xtics" <<endl;
            single_file_plot << "set ytics" <<endl;
            single_file_plot << "unset key" <<endl;
            single_file_plot << "plot './posteriorFile.txt' u 1:2 w l" <<endl;



//            G2->gnuplot_cmd("set multiplot"); // layout 3,3 rowsfirst");
//            G2->gnuplot_cmd("set origin 0.0,0.6");
//            G2->gnuplot_cmd("set size 0.3,0.3");
//            G2->gnuplot_cmd("set size ratio 0.4");
//            G2->gnuplot_cmd("set yrange [0:1]");
//            G2->gnuplot_cmd("set xrange [0:*]");
//            G2->gnuplot_cmd("set xlabel ''");
//            G2->gnuplot_cmd("set ylabel 'Posterior'");
//            G2->gnuplot_cmd("set xtics");
//            G2->gnuplot_cmd("set ytics");
//            G2->gnuplot_cmd("set notitle");
//            G2->gnuplot_cmd("unset key");
//            G2->gnuplot_cmd("plot './posteriorFile.txt' u 1:2 w l");


//            G2->gnuplot_cmd("unset key");
            single_file_plot << "set origin 0.0,0.3" <<endl;
            single_file_plot << "set nolabel" <<endl;
            single_file_plot << "set size 0.3,0.3" <<endl;
            single_file_plot << "set size ratio 0.4" <<endl;
            single_file_plot << "set yrange [0:*]" <<endl;
            single_file_plot << "set xrange [0:*]" <<endl;
            single_file_plot << "set xlabel ''" <<endl;
            single_file_plot << "set ylabel 'Likelihood'" <<endl;
            single_file_plot << "set xtics" <<endl;
            single_file_plot << "set ytics" <<endl;
            single_file_plot << "unset key" <<endl;
            single_file_plot << "plot './likelihoodFile.txt' u 1:2 w l" <<endl;


//            G2->gnuplot_cmd("set origin 0.0,0.3");
//            G2->gnuplot_cmd("set size 0.3,0.3");
//            G2->gnuplot_cmd("set size ratio 0.4");
//            G2->gnuplot_cmd("set yrange [0:*]");
//            G2->gnuplot_cmd("set xrange [0:*]");
//            G2->gnuplot_cmd("set xlabel ''");
//            G2->gnuplot_cmd("set ylabel 'Likelihood'");
//            G2->gnuplot_cmd("set xtics");
//            G2->gnuplot_cmd("set ytics");


//            G2->gnuplot_cmd("unset key");
//            G2->gnuplot_cmd("plot './likelihoodFile.txt' u 1:2 w l");



            //                G4.gnuplot_cmd("plot './hypoScoreFile.txt' u 1:2 w l");

            single_file_plot << "set origin 0.0,0.0" <<endl;
            single_file_plot << "set nolabel" <<endl;
            single_file_plot << "set size 0.3,0.3" <<endl;
            single_file_plot << "set size ratio 0.4" <<endl;
            single_file_plot << "set yrange [0:1]" <<endl;
            single_file_plot << "set xrange [0:*]" <<endl;
            single_file_plot << "set xlabel 'Node-Index'" <<endl;
            single_file_plot << "set ylabel 'Prior'" <<endl;
            single_file_plot << "set xtics" <<endl;
            single_file_plot << "set ytics" <<endl;
            single_file_plot << "unset key" <<endl;
            single_file_plot << "plot './predictedVectorFile.txt' u 1:2 w l" <<endl;

//            G2->gnuplot_cmd("unset key");

//            G2->gnuplot_cmd("set origin 0.0,0.0");

//            G2->gnuplot_cmd("set size 0.5,0.4");
//            G2->gnuplot_cmd("set size 0.3,0.3");
//            G2->gnuplot_cmd("set size ratio 0.4");
//            G2->gnuplot_cmd("set yrange [0:1]");
//            G2->gnuplot_cmd("set xrange [0:*]");

//            G2->gnuplot_cmd("set xlabel 'Node-Index'");
//            G2->gnuplot_cmd("set ylabel 'Prior'");
//            G2->gnuplot_cmd("set xtics");
//            G2->gnuplot_cmd("set ytics");

//            G2->gnuplot_cmd("unset key");
//            G2->gnuplot_cmd("plot './predictedVectorFile.txt' u 1:2 w l");


            priorFile.close();

            posteriorFile.close();
            hypoScoreFile.close();
            predictedVectorFile.close();
            likelihoodfile.close();
#endif

            double matching = 0.0;
            double maxMatching = 0.0;

if(NO_OF_VERIFICATION_CANDIDATES >= 1)
{

            maxMatching = 0.0;
            for(int i=0;i<candidateList.size();i++)
            {
                cout << candidateList[i] << "\t" << candidateProbability[i] << endl;
                if(candidateList[i] != (posteriorVector.size()-1) && candidateProbability[i] > LOOP_CLOSURE_HYPOTHESIS_THRESHOLD)
                {
                    // Check Spatial Arrangement with RANSAC
                    matching = surfMatching(dstDescriptors,nodeDescriptorVector[candidateList[i]],
                                                   dstKeypoint,nodeKeyPointVector[candidateList[i]]);
                    if(matching > maxMatching)
                    {
                        loopClosureCandidate = candidateList[i];
                        loopClosureProbability = candidateProbability[i];
                        maxMatching = matching;
                    }

                    cout << "SURF matching \t" << i << "\t" << imgCount+1 << "\t" << matching << "\t" << maxMatching << endl;
                }
            }
}
else
{



            // Check whether loop closure probable node satisfies the criteria of loop closure hypothesis threshold

            if(loopClosureProbability > LOOP_CLOSURE_HYPOTHESIS_THRESHOLD && loopClosureCandidate != (posteriorVector.size()-1))
            {

#if(COMPUTE_TIME)
                //      gettimeofday(&ransacStartTime,NULL);
                ransacStartTime = ros::Time::now();
#elif(OPENCV_COMPUTE_TIME)
                ransacStartTime = (double)cv::getTickCount();
#endif

                // Check Spatial Arrangement with RANSAC
                double matching = surfMatching(dstDescriptors,nodeDescriptorVector[loopClosureCandidate],
                                               dstKeypoint,nodeKeyPointVector[loopClosureCandidate]);

#if(COMPUTE_TIME)
                //      gettimeofday(&ransacEndTime,NULL);
                ransacEndTime = ros::Time::now();
#elif(OPENCV_COMPUTE_TIME)
                ransacEndTime = (double)cv::getTickCount();
#endif

                cout << "Matching \t" << matching << endl;

            }
            else
            {
                createNewNodeFlag = true;
            }
}


            if(maxMatching > MATCHING_THRESHOLD && !createNewNodeFlag)
            {
#if(COMPUTE_TIME)
                nodeUpdateStartTime = ros::Time::now();
#elif(OPENCV_COMPUTE_TIME)
                nodeUpdateStartTime = (double)cv::getTickCount();
#endif
                // Loop Closure Found
                createNewNodeFlag = false;
                loopClosureCount++;
                lastRobotPositionNode = loopClosureCandidate;

                // Writing probabilities
                //                    for(int i=0;i<predictedVector.size();i++)
                //                        priorFile << i << "\t" << predictedVector[i] << endl;

                //                    for(int i=0;i<likelihoodVector.size();i++)
                //                        likelihoodfile << i << "\t" << likelihoodVector[i] << endl;

                //                    for(int i=0;i<posteriorVector.size();i++)
                //                        posteriorFile << i << "\t" << posteriorVector[i] << endl;

                //                    for(int i=0;i<hypothesisScore.size();i++)
                //                        hypoScoreFile << i << "\t" << hypothesisScore[i] << endl;


                partitionDetails[loopClosureCandidate].push_back(imgCount+1);

                nodeSequenceInTime.push_back(loopClosureCandidate);




                cout << "Loop closure count \t" << loopClosureCount << endl;
                loopClosureFile << imgCount+1 << "\t" << nodeImageIndex[loopClosureCandidate] <<
                                   "\t" << loopClosureProbability << "\t" << maxMatching <<
                                   "\t" << likelihoodVector[loopClosureCandidate] << "\t"
                                << posteriorVector[loopClosureCandidate] << "\t" <<
                                   posteriorVector[posteriorVector.size()-1] << endl;
                vector<double> tempVec;
                tempVec.assign(posteriorVector.begin(),posteriorVector.end());
                posteriorVector.clear();

                //                    reassignPosterior(tempVec,posteriorVector);
                reassignPosteriorNew(tempVec,posteriorVector,lastRobotPositionNode);


                //                    exit(0);
#if(COMPUTE_TIME)
                nodeUpdateEndTime = ros::Time::now();
#elif(OPENCV_COMPUTE_TIME)
                nodeUpdateEndTime = (double)cv::getTickCount();
#endif


            }
            else
            {
                createNewNodeFlag = true;
            }
        }


#if(TEST_PHASE && VISUAL_PAIRS)
        if(createNewNodeFlag)
            newNodeStatus << imgCount << "\t" << 0 << endl;
        else
            loopStatusFile << imgCount << "\t" << 0 << endl;
#endif


        if(createNewNodeFlag)
        {
#if(COMPUTE_TIME)
            nodeUpdateStartTime = ros::Time::now();
#elif(OPENCV_COMPUTE_TIME)
            nodeUpdateStartTime = (double)cv::getTickCount();
#endif

            partitionDetails.push_back(vector<int>(0));
            partitionDetails[partitionCount].push_back(imgCount+1);

            loop_closure_node_number.push_back(partitionCount);
            node_i=1;


            nodeSequenceInTime.push_back(partitionCount);

            addNeighborsToNode(nodeSequenceInTime,neighborInfo);


            numOfHypothesis += 1;
            partitionCount++;

            //Storing the node number for displaying it on the plot.
            node_number=partitionCount;

            lastRobotPositionNode = -1;

            nodeKeyPointVector.push_back(dstKeypoint);
            nodeDescriptorVector.push_back(dstDescriptors);
            nodeImageIndex.push_back(imgCount+1);
            nodeImages.push_back(dstImg.clone());

#if(COMPUTE_TIME)
            nodeUpdateEndTime = ros::Time::now();
#elif(OPENCV_COMPUTE_TIME)
            nodeUpdateEndTime = (double)cv::getTickCount();
#endif


        }
        else
        {

            loopClosureDecision = true;
            loop_closure_node_number.push_back(loopClosureCandidate);
            node_j=1;
            lcdResultImage = nodeImages[loopClosureCandidate].clone();
            //Storing the node number for displaying it on the plot.
            node_number=loopClosureCandidate+1;    //Change Made
        }



        /* Section for adding new words found in the current image to
         * either a new node or existing loop closure node
         */

#if(COMPUTE_TIME)
        ros::Time wordTableUpdateStartTime = ros::Time::now();
#elif(OPENCV_COMPUTE_TIME)
        double wordTableUpdateStartTime = (double)cv::getTickCount();
#endif


        vector<int> tempWordListInCurrentImage;

        findUniqueElementsInArray(wordsPresentInCurrentImage,tempWordListInCurrentImage);

        if(createNewNodeFlag)
        {
            for(int i=0;i<newDescriptorInCurrentImage.rows;i++)
            {
                wordNodeIndexTable.push_back(vector<int>(1,partitionCount-1));

#if(HIERARCHICAL_NODES)
                dominantWordNodeIndexTable.push_back(vector<int>(1,-1));
#endif

            }


            for(int i=0;i<tempWordListInCurrentImage.size();i++)
            {
                wordNodeIndexTable[tempWordListInCurrentImage[i]].push_back(partitionCount-1);
            }

#if(HIERARCHICAL_NODES)
            for(int i=0;i<wordNodeIndexTable.size()-newDescriptorInCurrentImage.rows;i++)
            {
                if(wordFrequencyInCurrentImage[i] >= 2)
                    dominantWordNodeIndexTable[i].push_back(partitionCount-1);
            }
#endif

        }
        else
        {
            for(int i=0;i<newDescriptorInCurrentImage.rows;i++)
            {
                wordNodeIndexTable.push_back(vector<int>(1,loopClosureCandidate));
#if(HIERARCHICAL_NODES)
                dominantWordNodeIndexTable.push_back(vector<int>(1,-1));
#endif

            }


            for(int i=0;i<tempWordListInCurrentImage.size();i++)
            {
                wordNodeIndexTable[tempWordListInCurrentImage[i]].push_back(loopClosureCandidate);
            }

#if(HIERARCHICAL_NODES)
            for(int i=0;i<wordNodeIndexTable.size()-newDescriptorInCurrentImage.rows;i++)
            {
                if(wordFrequencyInCurrentImage[i] >= 2)
                    dominantWordNodeIndexTable[i].push_back(loopClosureCandidate);
            }
#endif
        }

#if(COMPUTE_TIME)
        ros::Time wordTableUpdateEndTime = ros::Time::now();
        ros::Time wordPairTableUpdateStartTime = ros::Time::now();
#elif(OPENCV_COMPUTE_TIME)
        double wordTableUpdateEndTime = (double)cv::getTickCount();
        double wordPairTableUpdateStartTime = (double)cv::getTickCount();
#endif

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /*
         * Section for adding new word pairs in the dictionary. These new word pairs are added to inverted
         * index table for each corresponding node.
         */

#if(VISUAL_PAIRS)


        for(int i=0;i<newWordPairInCurrentImage.size();i++)
        {
#if(EFFICIENT_WORD_PAIR_SEARCH)
            vector<int> temp ;
            temp.push_back(newWordPairInCurrentImage[i].second);

            temp.push_back(myWordPairsDictionaryTable.size());

            myWordPairsDictionaryTable.insert(std::pair<int,vector<int> >
                                              (newWordPairInCurrentImage[i].first,temp));
            temp.clear();
#else
            tempWordPairsDictionaryTable.push_back(tempNewWordPairInCurrentImage[i]);

#endif
        }

        // Condition for a new node, if a new node is created, a word-pair is pushed back in the dictionary.
        if(createNewNodeFlag)
        {
            for(int i=0;i<newWordPairInCurrentImage.size();i++)
                wordPairNodeIndexTable.push_back(vector<int>(1,partitionCount-1));


            for(int i=0;i<pairsPositionsFoundInwordPairs.size();i++)
                wordPairNodeIndexTable[pairsPositionsFoundInwordPairs[i]].push_back(partitionCount-1);
        }
        else
        {
            // If a loop closure is found, all the word pairs which are found in the current image are updated with the
            // current loop closure node.
            for(int i=0;i<newWordPairInCurrentImage.size();i++)
                wordPairNodeIndexTable.push_back(vector<int>(1,loopClosureCandidate));



            for(int i=0;i<pairsPositionsFoundInwordPairs.size();i++)
                wordPairNodeIndexTable[pairsPositionsFoundInwordPairs[i]].push_back(loopClosureCandidate);

        }

#endif


        // Section for adding new words to the dictionary (K-D tree)

#if(COMPUTE_TIME)
        ros::Time wordPairTableUpdateEndTime = ros::Time::now();

        ros::Time treeBuildStartTime;
        ros::Time treeBuildEndTime;
        ros::Time idfValueStartTime,idfValueEndTime;

#elif(OPENCV_COMPUTE_TIME)
        double wordPairTableUpdateEndTime = (double)cv::getTickCount();
        double treeBuildStartTime;
        double treeBuildEndTime;
        double idfValueStartTime,idfValueEndTime;
#endif




#if(COMPUTE_TIME)
        treeBuildStartTime = ros::Time::now();
#elif(OPENCV_COMPUTE_TIME)
        treeBuildStartTime = (double)cv::getTickCount();
#endif

        /*
             * Adding new words to dictionary in K-D tree incrementally using external flann library
             */


        appendMatrix(featureDataset,newDescriptorInCurrentImage);
        vectorOfNewWordsForFLANN.push_back(newDescriptorInCurrentImage);

        cout << "Image dictionary building \t" << imgCount << "\t" << featureDataset.rows << endl;


#if(EXTERNAL_FLANN_LIBRARY)
        //            delete[] flannFeatureDataset.ptr();
        if(imgCount == 0)
        {
            cout << "Building tree " << endl;
            flannFeatureDataset = flann::Matrix<float>((float*) featureDataset.data,featureDataset.rows,featureDataset.cols);
            index1->buildIndex(flannFeatureDataset);
        }
        else
        {

            if(newDescriptorInCurrentImage.rows > 0)
            {
            flannFeatureDataset = flann::Matrix<float>((float*) vectorOfNewWordsForFLANN[vectorOfNewWordsForFLANN.size()-1].data,
                    newDescriptorInCurrentImage.rows,newDescriptorInCurrentImage.cols);
            index1->addPoints(flannFeatureDataset,2);
            }
        }


#else
        index.build(featureDataset,KDparam,cvflann::FLANN_DIST_L2);
#endif

        tempFeatureDataset.release();



#if(COMPUTE_TIME)

        treeBuildEndTime = ros::Time::now();
        idfValueStartTime = ros::Time::now();
        ros::Time time1start = ros::Time::now();
        ros::Time time1end = ros::Time::now();
        ros::Time time2start = ros::Time::now();

#elif(OPENCV_COMPUTE_TIME)

        treeBuildEndTime = (double)cv::getTickCount();
        idfValueStartTime = (double)cv::getTickCount();
        double time1start = (double)cv::getTickCount();
        double time1end = (double)cv::getTickCount();
        double time2start = (double)cv::getTickCount();
#endif

        idfValueVector.clear();
        // Calculating idfvalues for each words
        for(int i=0;i<wordNodeIndexTable.size();i++)
        {
            idfValueVector.push_back(log(double(imgCount+1)/(wordNodeIndexTable[i].size())));
            //                idfValueVector.push_back(1);
        }


#if(COMPUTE_TIME)
        ros::Time time2end = ros::Time::now();
#elif(OPENCV_COMPUTE_TIME)
        double time2end = (double)cv::getTickCount();
#endif

        // Computing idf values fof word-pair vector at each iteration.

#if(VISUAL_PAIRS)
#if(COMPUTE_TIME)
        ros::Time time3start = ros::Time::now();
#elif(OPENCV_COMPUTE_TIME)
        double time3start = (double)cv::getTickCount();
#endif
        //            idfPairValueVector.clear();
        //            for(int i=0;i<wordPairNodeIndexTable.size();i++)
        //            {
        //                idfPairValueVector.push_back(log(double(imgCount+1)/(wordPairNodeIndexTable[i].size())));
        //            }

#if(COMPUTE_TIME)
        ros::Time time3end = ros::Time::now();
        cout << "ROUGHHHHHHH TIME \t" << (time1end-time1start).toSec() << "\t" <<(time2end-time2start).toSec() << "\t" <<
                (time3end-time3start).toSec() << endl;
        idfValueEndTime = ros::Time::now();
#elif(OPENCV_COMPUTE_TIME)
        double time3end = (double)cv::getTickCount();
        idfValueEndTime = (double)cv::getTickCount();
        cout << "ROUGHHHHHHH TIME \t" << (time1end-time1start)/cv::getTickFrequency() << "\t" <<
                (time2end-time2start)/cv::getTickFrequency() << "\t" << (time3end-time3start)/cv::getTickFrequency() << endl;
#endif



#endif



        // Clearning word pair related variables

#if(VISUAL_PAIRS)
        newWordPairInCurrentImage.clear();
        wordPairVector.clear();
        wordPairsInCurrentImage.clear();


        newWordsNotInDictionaryInCurrentImage.clear();
        newWordsLocationInImage.clear();
        newWordsRadiusInImage.clear();
        pairsPositionsFoundInwordPairs.clear();
#endif

        // Clearning word related variables

        newDescriptorInCurrentImage.release();
        wordsPresentInCurrentImage.clear();
        tempWordListInCurrentImage.clear();


#if(HIERARCHICAL_NODES)

//        vector<int> maximumdominantWordsMatchingWordCount(wordsPresentInCurrentImage.size(),0);

//        // Clustering nodes on the basis of common dominant words present in the dictionary.
//        for(int i=0;i<wordsPresentInCurrentImage.size();i++)
//        {
//            for(int j=1;j<dominantWordNodeIndexTable[wordsPresentInCurrentImage[i]].size();i++)
//            {
//                int number = dominantWordNodeIndexTable[wordsPresentInCurrentImage[i]][j];
//                maximumdominantWordsMatchingWordCount[number]++;
//            }
//        }


#endif



        imgCount++;



#if(COMPUTE_TIME)
        //        gettimeofday(&loopEndTime,NULL);
        loopEndTime = ros::Time::now();
        loopDiffTime = (loopEndTime - loopStartTime).toSec();
        descriptorSearchDifftime = (descriptorSearchEndTime - descriptorSearchStartTime).toSec();
        surfDetectDiffTime = (surfDetectEndTime - surfDetectStartTime).toSec();
        wordPairSearchDiffTime = (wordPairSearchEndtime-wordPairSearchStartTime).toSec();
        transitionDiffTime = (transitionCalcEndTime - transitionCalcStartTime).toSec();
        matchScoreDiffTime = (matchScoreEndTime - matchScoreStartTime).toSec();
        ransacDiffTime = (ransacEndTime - ransacStartTime).toSec();
        nodeUpdateDiffTime = (nodeUpdateEndTime-nodeUpdateStartTime).toSec();
        wordTableUpdateDiffTime = (wordTableUpdateEndTime-wordTableUpdateStartTime).toSec();
        wordPairTableUpdateDiffTime = (wordPairTableUpdateEndTime-wordPairTableUpdateStartTime).toSec();
        treeBuildDiffTime =(treeBuildEndTime-treeBuildStartTime).toSec();
        idfValueDiffTime = (idfValueEndTime-idfValueStartTime).toSec();
#elif(OPENCV_COMPUTE_TIME)
        loopEndTime = (double)cv::getTickCount();
        loopDiffTime = (loopEndTime - loopStartTime)/cv::getTickFrequency();
        descriptorSearchDifftime = (descriptorSearchEndTime - descriptorSearchStartTime)/cv::getTickFrequency();
        surfDetectDiffTime = (surfDetectEndTime - surfDetectStartTime)/cv::getTickFrequency();
        wordPairSearchDiffTime = (wordPairSearchEndtime-wordPairSearchStartTime)/cv::getTickFrequency();
        transitionDiffTime = (transitionCalcEndTime - transitionCalcStartTime)/cv::getTickFrequency();
        matchScoreDiffTime = (matchScoreEndTime - matchScoreStartTime)/cv::getTickFrequency();
        ransacDiffTime = (ransacEndTime - ransacStartTime)/cv::getTickFrequency();
        nodeUpdateDiffTime = (nodeUpdateEndTime-nodeUpdateStartTime)/cv::getTickFrequency();
        wordTableUpdateDiffTime = (wordTableUpdateEndTime-wordTableUpdateStartTime)/cv::getTickFrequency();
        wordPairTableUpdateDiffTime = (wordPairTableUpdateEndTime-wordPairTableUpdateStartTime)/cv::getTickFrequency();
        treeBuildDiffTime =(treeBuildEndTime-treeBuildStartTime)/cv::getTickFrequency();
        idfValueDiffTime = (idfValueEndTime-idfValueStartTime)/cv::getTickFrequency();
#endif

        avgsurfDetectDiffTime = (double)(((frameNumber-1) * avgsurfDetectDiffTime) + surfDetectDiffTime)/(frameNumber);
        avgtransitionDiffTime = (double)(((frameNumber-1) * avgtransitionDiffTime) + transitionDiffTime)/(frameNumber);
        avgdescriptorSearchDifftime = (double)(((frameNumber-1) * avgdescriptorSearchDifftime) + descriptorSearchDifftime )/(frameNumber);
        avgmatchScoreDiffTime = (double)(((frameNumber-1) * avgmatchScoreDiffTime) + matchScoreDiffTime )/(frameNumber);
        avgransacDiffTime = (double)(((frameNumber-1) * avgransacDiffTime) + ransacDiffTime )/(frameNumber);
        avgloopDiffTime = (double)(((frameNumber-1) * avgloopDiffTime) + loopDiffTime )/(frameNumber);
        avgWordPairSearchDiffTime = (double)(((frameNumber-1) * avgWordPairSearchDiffTime) + wordPairSearchDiffTime)/(frameNumber);
        avgNodeUpdateTime = (double)(((frameNumber-1) * avgNodeUpdateTime) + nodeUpdateDiffTime)/(frameNumber);
        avgwordTableUpdateTime = (double)(((frameNumber-1) * avgwordTableUpdateTime) + wordTableUpdateDiffTime)/(frameNumber);
        avgWordPairTableUpdateTime = (double)(((frameNumber-1) * avgWordPairTableUpdateTime) + wordPairTableUpdateDiffTime)/(frameNumber);
        avgTreeBuildTime = (double)(((frameNumber-1) * avgTreeBuildTime) + treeBuildDiffTime)/(frameNumber);
        avgIdfCalculationTime = (double)(((frameNumber-1) * avgIdfCalculationTime) + idfValueDiffTime)/(frameNumber);

        timeFile << loopDiffTime << "\t" << avgloopDiffTime << "\t" <<
                    surfDetectDiffTime << "\t" << avgsurfDetectDiffTime << "\t" <<
                    descriptorSearchDifftime << "\t" << avgdescriptorSearchDifftime << "\t" <<
                    wordPairSearchDiffTime << "\t" << avgWordPairSearchDiffTime << "\t" <<
                    transitionDiffTime << "\t" << avgtransitionDiffTime << "\t" <<
                    matchScoreDiffTime << "\t" << avgmatchScoreDiffTime << "\t" <<
                    nodeUpdateDiffTime << "\t" << avgNodeUpdateTime << "\t" <<
                    ransacDiffTime << "\t" << avgransacDiffTime << "\t" <<
                    wordTableUpdateDiffTime << "\t" << avgwordTableUpdateTime << "\t" <<
                    wordPairTableUpdateDiffTime << "\t" << avgWordPairTableUpdateTime << "\t" <<
                    treeBuildDiffTime << "\t" << avgTreeBuildTime << "\t" <<
                    idfValueDiffTime << "\t" << avgIdfCalculationTime << "\t" <<

            #if(VISUAL_PAIRS && COMPUTE_TIME)
                    onlyPairSearchTime << "\t" <<
                    (wpMakeHistEndTime-wpMakeHistStartTime).toSec() << endl;
#elif(VISUAL_PAIRS && OPENCV_COMPUTE_TIME)
                    onlyPairSearchTime << "\t" <<
                    (wpMakeHistEndTime-wpMakeHistStartTime)/cv::getTickFrequency() << endl;
#else
                    endl;
#endif

#if(HIERARCHICAL_NODES)
        // Publishing the dominant words Vs Nodes
        cout << "Publishing the dominant vs word information \t" << endl;
        for(int i=0;i<dominantWordNodeIndexTable.size();i++)
        {
            if(dominantWordNodeIndexTable[i].size() > 1)
            {
            cout << i << "\t";
            for(int j=0;j<dominantWordNodeIndexTable[i].size();j++)
            {
                cout << dominantWordNodeIndexTable[i][j] << "\t";
            }
            cout << endl;
            }
        }

#endif



        if(WAITKEY)
        {
            while(getchar()!='\n')
            {}
        }

    return;
}

/*
 * Callback function for loop closure detection based on images collected through kinect
 * It subscribes kinect images and publishes loop closure decision and loop closure image
 * based on the algorithm call.
 */
void BowpMap::loopClosureCallBack(const sensor_msgs::ImageConstPtr &image)
{
    if (image_capture_flag==1)
    {

     loopClosureDecision = false;
     std_msgs::Bool loopClosureDecision_stdmsgs;

     cv_bridge::CvImagePtr bridge;
     try
     {
         bridge=cv_bridge::toCvCopy(image,"bgr8");
     }
     catch (cv_bridge::Exception& e)
     {
          ROS_ERROR("Failed to transform color image.");
         return;
     }


     cv::Mat cameraImage = cv::Mat(bridge->image);


     cv::Mat loopClosureImage;

     identifyPlaceForCameraImage(cameraImage,loopClosureImage);

     loopClosureDecision_stdmsgs.data = loopClosureDecision;

     loopClosureEventPublisher.publish(loopClosureDecision_stdmsgs);




     cv::Mat finalImage;
     cv::Mat plotFinalImage;

     if(!loopClosureDecision)
     {


     loopClosureImage = cv::Mat(cameraImage.rows,cameraImage.cols,CV_8UC3,cv::Scalar(255,255,255));

     cv::vconcat(cameraImage,loopClosureImage,plotFinalImage);


     }
     else
     {

         cv::vconcat(cameraImage,loopClosureImage,plotFinalImage);

     }

     // Creating a partition in output to show node number

     cv::Mat node_number_disp_panel = cv::Mat(60,plotFinalImage.cols,CV_8UC3,cv::Scalar(255,255,255));
     cv::vconcat(node_number_disp_panel,plotFinalImage,plotFinalImage);


     // Write FinalVerticalConcatinatedImage in the folder

     string result,result1 ("Node Index = "),result2 ("-----------");//string which will contain the result
     stringstream convert; // stringstream used for the conversion
     convert << node_number;//add the value of node number to the characters in the stream
     result = convert.str();//set Result to the content of the stream
     result = result1 + result;
     cv::putText(plotFinalImage,result,cv::Point(plotFinalImage.cols/3.5,30),2,1.0,cv::Scalar(0,0,0),2,8,0);//putting the node no. in the figure.
     cv::putText(plotFinalImage,result2,cv::Point(plotFinalImage.cols/3.5,44),2,1.0,cv::Scalar(0,0,0),1,8,0);
     char s11[200];
     sprintf(s11,"./QueryImage/%d.jpg",frameNumber);
     cv::imwrite(s11,plotFinalImage);

#if(GNUPLOTS)

            single_file_plot.close();

            G2->gnuplot_cmd("load './single_file_plot.gp'");

            G2->gnuplot_cmd("set origin -0.10,0.10");
            G2->gnuplot_cmd("set size 1,0.8");
            G2->gnuplot_cmd("unset key");
            G2->gnuplot_cmd("unset xtics");
            G2->gnuplot_cmd("unset ytics");
            G2->gnuplot_cmd("set size ratio -1");
            G2->gnuplot_cmd("set autoscale y");
            G2->gnuplot_cmd("set xlabel ''");
            G2->gnuplot_cmd("set ylabel ''");
            G2->gnuplot_cmd("set title 'Query Image'");
            char s12[200];
            sprintf(s12,"./QueryImage/%d.jpg",frameNumber);
            G2->gnuplot_image(s12,"jpg");

            G2->gnuplot_cmd("load './new_node_no.gp'");

            single_file_plot.close();


#endif




     cv::hconcat(cameraImage,loopClosureImage,finalImage);



     cv::Mat outputPanel = cv::Mat(100,finalImage.cols,CV_8UC3,cv::Scalar(255,255,255));

     cv::vconcat(finalImage,outputPanel,finalImage);

     char s1[200],s2[200];
     sprintf(s1,"Query Image");

     if(loopClosureDecision)
         sprintf(s2,"LoopClosure Image");
     else
         sprintf(s2,"Loop Closure Not Found");




     cv::putText(finalImage,s1,cv::Point(finalImage.cols/5,finalImage.rows-70),2,1.0,cv::Scalar(255,0,0),1,8,0);
     cv::putText(finalImage,s2,cv::Point(2*finalImage.cols/3,finalImage.rows-70),2,1.0,cv::Scalar(255,0,0),1,8,0);



     cv::resize(finalImage,finalImage,cv::Size(finalImage.cols/3,finalImage.rows/3));

     char s[200];
     sprintf(s,"./Results/%d.jpg",frameNumber);
     cv::imwrite(s,finalImage);



     sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),"bgr8", finalImage).toImageMsg();
     loopClosureImagePublisher.publish(msg);

     image_capture_flag=0;

     return;
    }
}


/* This function is to calculate the displacement of the robot in order to capture image at fixed intervals */

void BowpMap::odometryCallback(const nav_msgs::Odometry::ConstPtr &odometry)
{

    cv::Point2f pt;

    double roll, pitch;
    tf::Quaternion q;
    tf::quaternionMsgToTF(odometry->pose.pose.orientation, q);
    tf::quaternionMsgToTF(odometry->pose.pose.orientation, q);
    tf::Matrix3x3(q).getRPY(roll,pitch,yaw);

    if (yaw<0)
    {
        yaw = yaw+(2*3.1416);
    }

    if (update_initial_pos_flag==0)
    {
        initial_yaw=yaw;
        initial_pos_a=odometry->pose.pose.position.x;
        initial_pos_b=odometry->pose.pose.position.y;
        update_initial_pos_flag=1;


        pt.x = initial_pos_a;
        pt.y = initial_pos_b;

        odom_reading_for_new_nodes_xy.push_back(pt);
    }
    if (1)
    {

//      To save the odom reading in .txt file.

        if(GNUPLOTS)
        {

        ofstream new_nodes_xy,new_node_no,test_file;
        new_nodes_xy.open("./new_nodes_xy.txt");
        new_node_no.open("./new_node_no.gp");
        test_file.open("./test_file.txt");
        if(!new_nodes_xy.is_open())
        {
           cerr << "Error opening new_nodes_xy.txt" << endl;
           exit(0);
        }
        if(!new_node_no.is_open())
        {
           cerr << "Error opening new_node_no.txt" << endl;
           exit(0);
        }
        if(!test_file.is_open())
        {
           cerr << "Error opening test_file.txt" << endl;
           exit(0);
        }
        float buffer_x,buffer_y;
        int buffer_node_number;


        new_node_no << "set origin 0.13,0.06"<<endl;
        new_node_no << "set size 1.25,.75" << endl;
        new_node_no << "set size ratio 1"<<endl;
        new_node_no << "set xlabel 'X (m)'"<<endl;
        new_node_no << "set ylabel 'Y (m)'"<<endl;
        new_node_no << "set xrange [*:*]"<<endl;
        new_node_no << "set yrange [*:*]"<<endl;
        new_node_no << "set xtics"<<endl;
        new_node_no << "set ytics"<<endl;
        new_node_no << "set notitle"<<endl;
        new_node_no << "unset key"<<endl;
        new_node_no << "set style line 1 lc rgb 'red' lw 2"<<endl;
        new_node_no << "set style line 2 lc rgb 'blue'  lw 2"<<endl;
        new_node_no << "set nolabel" <<endl;

        if(node_i == 1 || node_j == 1)
        {
            for(int j=0; j<loop_closure_node_number.size() ; j++)
            {
                test_file << odom_reading_for_new_nodes_xy[j].x << "\t" << odom_reading_for_new_nodes_xy[j].y << "\t" << j << "\t "<< loop_closure_node_number[j] << endl;
            }

       if (1)
       {
        int lastMax = 1;
        for (int i=0; i<loop_closure_node_number.size() ; i++)
        {            
            if (lastMax == loop_closure_node_number[i]+1)
            {
                if(!isnan (odom_reading_for_new_nodes_xy[i].x) && !isnan (odom_reading_for_new_nodes_xy[i].y) && i < loop_closure_node_number.size()-1)
                {
                  if(i==loop_closure_node_number.size()-2)
                  {
                    buffer_x=odom_reading_for_new_nodes_xy[i].x;
                    buffer_y=odom_reading_for_new_nodes_xy[i].y;
                    buffer_node_number=loop_closure_node_number[i]+1;
                  }
                  new_nodes_xy << odom_reading_for_new_nodes_xy[i].x <<  setprecision(5) << "\t" << odom_reading_for_new_nodes_xy[i].y <<  setprecision(5) << "\t" << loop_closure_node_number[i]+1 << endl;
                  new_node_no << "set label '" << loop_closure_node_number[i]+1 << "' at " << odom_reading_for_new_nodes_xy[i].x << "," << odom_reading_for_new_nodes_xy[i].y<<endl;
                }
                else if(!isnan (odom_reading_for_new_nodes_xy[i].x) && !isnan (odom_reading_for_new_nodes_xy[i].y) && i == loop_closure_node_number.size()-1)
                {
                  new_nodes_xy << endl << endl << buffer_x << setprecision(5) << "\t" << buffer_y << setprecision(5) << "\t" << buffer_node_number << endl << endl << endl << buffer_x << setprecision(5) << "\t" << buffer_y << setprecision(5) << "\t" << buffer_node_number << endl << odom_reading_for_new_nodes_xy[i].x << setprecision(5) << "\t" << odom_reading_for_new_nodes_xy[i].y << setprecision(5) << "\t" << loop_closure_node_number[i]+1 << endl << odom_reading_for_new_nodes_xy[i].x <<  setprecision(5) << "\t" << odom_reading_for_new_nodes_xy[i].y <<  setprecision(5) << "\t" << loop_closure_node_number[i]+1 << endl;
                  new_node_no << "set label '" << loop_closure_node_number[i]+1 << "' at " << odom_reading_for_new_nodes_xy[i].x << "," << odom_reading_for_new_nodes_xy[i].y<<endl;
                }
             lastMax ++;
            }
            else
            {
                if(!isnan (odom_reading_for_new_nodes_xy[loop_closure_node_number[i]].x) && !isnan (odom_reading_for_new_nodes_xy[loop_closure_node_number[i]].y) && i < loop_closure_node_number.size()-1)
                {
                   if(i==loop_closure_node_number.size()-2)
                   {
                       buffer_x=odom_reading_for_new_nodes_xy[loop_closure_node_number[i]].x;
                       buffer_y=odom_reading_for_new_nodes_xy[loop_closure_node_number[i]].y;
                       buffer_node_number=loop_closure_node_number[i]+1;
                   }
                   new_nodes_xy << odom_reading_for_new_nodes_xy[loop_closure_node_number[i]].x <<  setprecision(5) << "\t" << odom_reading_for_new_nodes_xy[loop_closure_node_number[i]].y <<  setprecision(5) << "\t" << loop_closure_node_number[i]+1 << endl;
                   new_node_no << "set label '" << loop_closure_node_number[i]+1 << "' at " << odom_reading_for_new_nodes_xy[loop_closure_node_number[i]].x << "," << odom_reading_for_new_nodes_xy[loop_closure_node_number[i]].y << "left norotate"<<endl;
                }
                else if(!isnan (odom_reading_for_new_nodes_xy[loop_closure_node_number[i]].x) && !isnan (odom_reading_for_new_nodes_xy[loop_closure_node_number[i]].y) && i == loop_closure_node_number.size()-1)
                {
                   new_nodes_xy << endl << endl << buffer_x << setprecision(5) << "\t" << buffer_y << setprecision(5) << "\t" << buffer_node_number << endl << endl << endl << buffer_x << setprecision(5) << "\t" << buffer_y << setprecision(5) << "\t" << buffer_node_number << endl << odom_reading_for_new_nodes_xy[loop_closure_node_number[i]].x << setprecision(5) << "\t" << odom_reading_for_new_nodes_xy[loop_closure_node_number[i]].y << setprecision(5) << "\t" << loop_closure_node_number[i]+1 << endl << odom_reading_for_new_nodes_xy[loop_closure_node_number[i]].x <<  setprecision(5) << "\t" << odom_reading_for_new_nodes_xy[loop_closure_node_number[i]].y <<  setprecision(5) << "\t" << loop_closure_node_number[i]+1 << endl;
                   new_node_no << "set label '" << loop_closure_node_number[i]+1 << "' at " << odom_reading_for_new_nodes_xy[loop_closure_node_number[i]].x << "," << odom_reading_for_new_nodes_xy[loop_closure_node_number[i]].y << "left norotate"<<endl;
                }
            }
        }
       }

        new_node_no << "show label"<<endl;

        new_node_no << "plot './new_nodes_xy.txt'  index 0 with lines ls 2 ,'./new_nodes_xy.txt'  index 2 with lines ls 1 ,'./new_nodes_xy.txt'  index 0 u 1:2:(.05) with circle fc rgb 'blue', './new_nodes_xy.txt'  index 2 u 1:2:(0.05) with circle fc rgb 'red' , './new_nodes_xy.txt'  index 1 u 1:2:(0.05) with circle fc rgb 'blue'"<<endl;

        new_nodes_xy.close();
        new_node_no.close();
        test_file.close();
       }
    }
    }

    if ((sqrt(pow((initial_pos_a-odometry->pose.pose.position.x),2) + pow((initial_pos_b-odometry->pose.pose.position.y),2)) >= 0.5) || abs(yaw-initial_yaw) >= (3.1416/6))
    {
        update_initial_pos_flag=0;
        image_capture_flag=1;
    }
}
