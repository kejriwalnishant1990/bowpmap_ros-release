#ifndef PARAMTERS_H
#define PARAMTERS_H

#include <ros/ros.h>



class Parameter{
private:
protected:
public:


        bool GNUPLOTS;
        bool WAITKEY;
        bool VISUAL_PAIRS;
        bool TEST_PHASE;
        bool COMPUTE_TIME;
        bool ONLINE_DICTIONARY;
        bool EFFICIENT_WORD_PAIR_SEARCH;
        bool OPENCV_COMPUTE_TIME;
        bool HIERARCHICAL_NODES;
        bool KDTREE;
        bool LSH;
        bool OPENCV_FEATURE_DETECTOR;
        bool SHOW_LOOP_CLOSURE_IMAGE;
        bool flann_library;

    Parameter()
    {
                flann_library = true;

//                nImage.param("GNUPLOTS",GNUPLOTS,false);
//                nImage.param("WAITKEY",WAITKEY,false);
//                nImage.param("VISUAL_PAIRS",VISUAL_PAIRS,true);
//                nImage.param("TEST_PHASE",TEST_PHASE,false);
//                nImage.param("COMPUTE_TIME",COMPUTE_TIME,false);
//                nImage.param("ONLINE_DICTIONARY",ONLINE_DICTIONARY,true);
//                nImage.param("EFFICIENT_WORD_PAIR_SEARCH",EFFICIENT_WORD_PAIR_SEARCH,false);
//                nImage.param("OPENCV_COMPUTE_TIME",OPENCV_COMPUTE_TIME,true);

//                nImage.param("HIERARCHICAL_NODES",HIERARCHICAL_NODES,false);
//                nImage.param("KDTREE",KDTREE,true);
//                nImage.param("LSH",LSH,false);
//                nImage.param("OPENCV_FEATURE_DETECTOR",OPENCV_FEATURE_DETECTOR,false);
//                nImage.param("SHOW_LOOP_CLOSURE_IMAGE",SHOW_LOOP_CLOSURE_IMAGE,true);
                nImage.param("flann_library",flann_library,true);

                if(GNUPLOTS)
                {
#define GNUPLOTS                    1           // Set 1 for plotting the results on gnuplot
                }
                else
                {
#define GNUPLOTS                    0
                }

                if(WAITKEY)
                {
#define WAITKEY                     1           // Set 1 for enabling wait at the end of every frame
                }
                else
                {
#define WAITKEY                     0
                }

                if(COMPUTE_TIME)
                {
#define COMPUTE_TIME                1           // Set 1 for computing time complexity

                }
                else
                {
#define COMPUTE_TIME                0           // Set 1 for computing time complexity

                }

                if(VISUAL_PAIRS)
                {
#define VISUAL_PAIRS                1           // Set 1 for results with word-pair dictionary

                }
                else
                {
#define VISUAL_PAIRS                0           // Set 1 for results with word-pair dictionary

                }

                if(TEST_PHASE)
                {
#define TEST_PHASE                  1           // Set 1 for running modules in the test phase, 0 otherwise

                }
                else
                {
#define TEST_PHASE                  0           // Set 1 for running modules in the test phase, 0 otherwise

                }

                if(ONLINE_DICTIONARY)
                {
#define ONLINE_DICTIONARY           1           // Set 1 for using incremental online generation

                }
                else
                {
#define ONLINE_DICTIONARY           0           // Set 1 for using incremental online generation

                }

                if(EFFICIENT_WORD_PAIR_SEARCH)
                {
#define EFFICIENT_WORD_PAIR_SEARCH  1       // Set 1 for efficient word-pair search

                }
                else
                {
#define EFFICIENT_WORD_PAIR_SEARCH  0       // Set 1 for efficient word-pair search

                }

                if(flann_library)
                {
#define EXTERNAL_FLANN_LIBRARY      1          // Set 1 for using external FLANN library

                }
                else
                {
#define EXTERNAL_FLANN_LIBRARY      0          // Set 1 for using external FLANN library

                }

                if(OPENCV_COMPUTE_TIME)
                {
#define OPENCV_COMPUTE_TIME         1           //set 1 for using opencv library for computing execution time

                }
                else
                {
#define OPENCV_COMPUTE_TIME         10          //set 1 for using opencv library for computing execution time

                }

                if(HIERARCHICAL_NODES)
                {
#define HIERARCHICAL_NODES          1           // Set 1 for enabling hierarchical clustering

                }
                else
                {
#define HIERARCHICAL_NODES          0           // Set 1 for enabling hierarchical clustering

                }


                if(KDTREE)
                {
#define KDTREE                      1           // Set 1 for kd tree based indexing

                }
                else
                {
#define KDTREE                      0           // Set 1 for kd tree based indexing
                }

                if(LSH)
                {
#define LSH                         1          // Set 1 for LSh based indexing

                }
                else
                {
#define LSH                         0           // Set 1 for LSh based indexing

                }


                if(OPENCV_FEATURE_DETECTOR)
                {
#define OPENCV_FEATURE_DETECTOR     1
                }
                else
                {
#define OPENCV_FEATURE_DETECTOR     0

                }


                if(SHOW_LOOP_CLOSURE_IMAGE)
                {
#define SHOW_LOOP_CLOSURE_IMAGE     1
                }
                else
                {
#define SHOW_LOOP_CLOSURE_IMAGE     0

                }
    }
};

#endif // PARAMTERS_H
