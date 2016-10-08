#ifndef AUGTOPOMAP_H
#define AUGTOPOMAP_H

#include <opencv2/opencv.hpp>

#define RANSAC_THRESHOLD    1
#define MAX_INLIER_DIST     10
#define NEIGHBOR_THRESHOLD  8



using namespace std;

void appendMatrix( cv::Mat &originalMat,const cv::Mat& matToBeAppended )
{

    if(matToBeAppended.rows == 0)
        return;

    if(! originalMat.empty() )
    {
        assert( originalMat.cols == matToBeAppended.cols ) ;
        assert( originalMat.type() == matToBeAppended.type() ) ;


        cv::Mat newTemp( originalMat.rows+ matToBeAppended.rows , matToBeAppended.cols,matToBeAppended.type() ) ;

        int i ;
        for( i=0;i< originalMat.rows ; i++)
        {
            cv::Mat rowI = newTemp.row(i) ;
            //originalMat.row(i).copyTo( newTemp.row(i) );
            originalMat.row(i).copyTo( rowI );

        }
        for(int j=0; j< matToBeAppended.rows ; j++ )
        {
            cv::Mat rowJpluI =  newTemp.row( j+i ) ;
            //matToBeAppended.row(j).copyTo( newTemp.row( j+i ) );
            matToBeAppended.row(j).copyTo( rowJpluI );
        }

        originalMat = newTemp ;

    }
    else
    {
        originalMat = matToBeAppended ;
    }
    return ;

}

/*
  Build tree Index
  */

void buildTreeIndex(cv::Mat &features,cv::flann::Index &index)
{

    index.build(features,cv::flann::KDTreeIndexParams(1),cvflann::FLANN_DIST_L2);
    return;
}

/*
  Function for finding the unique elements in an array
  */

void findUniqueElementsInArray(vector<int> &inputArray, vector<int> &outputArray)
{
    vector<bool> maskArray(inputArray.size(),true);
    for(int i=0;i<inputArray.size();i++)
    {
        if(maskArray[i])
        {
            for(int j=i+1;j<inputArray.size();j++)
            {
                if(inputArray[i] == inputArray[j])
                {
                    maskArray[j] = false;
                }
            }
            outputArray.push_back(inputArray[i]);
        }
    }

    return;
}

/*
  Function for returning a pair insertion position from a multimap
  */

void findPairPositionInMultimap(std::multimap<int, vector<int> > &myWordPairsDictionaryTable,std::pair<int,int> testPair,int &pairPosition)
{
    std::pair <std::multimap<int,vector<int > >::iterator, std::multimap<int, vector<int> >::iterator> ret;
    ret = myWordPairsDictionaryTable.equal_range(testPair.first);

    for(std::multimap<int,vector<int> >::iterator it=ret.first; it !=ret.second; ++it)
    {
        if(it->second[0] == testPair.second)
        {
            pairPosition = it->second[1];
            break;
        }
    }

    return;
}

/*
  Function to find unique pairs in a vector of pairs
  */
void findUniquePairInVectorOfPairs(vector< pair<int,int> > &newWordPairInCurrentImage,vector< pair<int,int> > &tempNewWordPairInCurrentImage)
{
    vector<bool> maskArray(newWordPairInCurrentImage.size(),true);

    for(int i=0;i< newWordPairInCurrentImage.size();i++)
    {
        if(maskArray[i])
        {
            for(int j=i+1;j<newWordPairInCurrentImage.size();j++)
            {
                if(newWordPairInCurrentImage[i].first == newWordPairInCurrentImage[j].first
                        && newWordPairInCurrentImage[i].second == newWordPairInCurrentImage[j].second )
                {
                    maskArray[j] = false;
                }
            }
            tempNewWordPairInCurrentImage.push_back(newWordPairInCurrentImage[i]);
        }
    }

    return;
}
/*
  Function to return unique entries in the multimap
  Output : multimap
  Input : multimap
  */
void findUniqueWordPairs(std::multimap<int,int> &newWordPairInCurrentImage,std::multimap<int,int> &
                         tempNewWordPAirInCurrentImage)
{

    std::multimap<int,int>::iterator it;
    for(it=newWordPairInCurrentImage.begin();it!=newWordPairInCurrentImage.end();++it)
    {
        bool myflag = false;
        std::pair <std::multimap<int,int>::iterator, std::multimap<int,int>::iterator> ret;
        ret = tempNewWordPAirInCurrentImage.equal_range(it->first);
        for(std::multimap<int,int>::iterator iter=ret.first; iter !=ret.second; ++iter)
        {
            if(it->second == iter->second)
                myflag = true;
        }
        if(!myflag)
        tempNewWordPAirInCurrentImage.insert(std::pair<int,int>(it->first,it->second));
    }
    return;
}

/*
  Function to find pair position the array of pairs
  */
void findPairInPairVector(vector<pair<int,int> > &wordPairsDictionaryTable,pair<int,int> &testPair,int &pairPosition)
{
    for(int i=0;i<wordPairsDictionaryTable.size();i++)
    {
        if(wordPairsDictionaryTable[i].first == testPair.first && wordPairsDictionaryTable[i].second == testPair.second)
        {
            pairPosition = i;
            break;
        }
    }
    return;
}


/*
  This function is written for descriptor space matching between descriptors of
  two images.
  */

void crossCheckMatching( cv::Ptr<cv::DescriptorMatcher>& descriptorMatcher,
                         const cv::Mat& descriptors1, const cv::Mat& descriptors2,
                         vector<cv::DMatch>& filteredMatches12, int knn=1 )
{
    filteredMatches12.clear();
    assert(!descriptors1.empty());
    vector<vector<cv::DMatch> > matches12, matches21;
    descriptorMatcher->knnMatch( descriptors1, descriptors2, matches12, knn );
    descriptorMatcher->knnMatch( descriptors2, descriptors1, matches21, knn );
    for( size_t m = 0; m < matches12.size(); m++ )
    {
        bool findCrossCheck = false;
        for( size_t fk = 0; fk < matches12[m].size(); fk++ )
        {
            cv::DMatch forward = matches12[m][fk];

            for( size_t bk = 0; bk < matches21[forward.trainIdx].size(); bk++ )
            {
                cv::DMatch backward = matches21[forward.trainIdx][bk];
                if( backward.trainIdx == forward.queryIdx )
                {
                    filteredMatches12.push_back(forward);
                    findCrossCheck = true;
                    break;
                }
            }
            if( findCrossCheck ) break;
        }
    }
}


/*
  This function is written for finding the surf matching percentage between two images.
  */
double surfMatching(cv::Mat const &descriptor1,cv::Mat const &descriptor2,vector<cv::KeyPoint> &keypoint1,vector<cv::KeyPoint>
                     & keypoint2)
{
    double percentage_match = 0.0;
    cv::Ptr<cv::DescriptorMatcher> descriptorMatcher = cv::DescriptorMatcher::create("FlannBased");
    vector<cv::DMatch> filteredMatches;

    cout << descriptor1.rows << "\t" << descriptor2.rows << endl;

    if(descriptor1.rows <= 5 || descriptor2.rows <= 5)
        return(0.0);

    // Find matching in descriptor space
    crossCheckMatching(descriptorMatcher,descriptor1,descriptor2,filteredMatches,1);

    if(filteredMatches.size() < 5)
        return(0.0);


    vector<int> queryIdxs,  trainIdxs;
    for( size_t i = 0; i < filteredMatches.size(); i++ )
    {
        queryIdxs.push_back(filteredMatches[i].queryIdx);
        trainIdxs.push_back(filteredMatches[i].trainIdx);
    }



    // Homography matrix
    cv::Mat H12;
    if( RANSAC_THRESHOLD >= 0 )
    {
        vector<cv::Point2f> points1; cv::KeyPoint::convert(keypoint1, points1, queryIdxs);
        vector<cv::Point2f> points2; cv::KeyPoint::convert(keypoint2, points2, trainIdxs);
        H12 = cv::findHomography( cv::Mat(points1), cv::Mat(points2), CV_RANSAC, RANSAC_THRESHOLD );
    }

    if( !H12.empty() )
    {
        vector<char> matchesMask( filteredMatches.size(), 0 );
        vector<cv::Point2f> points1; cv::KeyPoint::convert(keypoint1, points1, queryIdxs);
        vector<cv::Point2f> points2; cv::KeyPoint::convert(keypoint2, points2, trainIdxs);

        cv::Mat points1t; cv::perspectiveTransform(cv::Mat(points1), points1t, H12);
        int count = 0;
        for( size_t i1 = 0; i1 < points1.size(); i1++ )
        {
            if( ( norm(points2[i1] - points1t.at<cv::Point2f>((int)i1,0)) <= int(MAX_INLIER_DIST) ))
            {
                count++;
                matchesMask[i1] = 1;
            }
        }

        percentage_match = (double)(2*count*100)/(descriptor1.rows+descriptor2.rows);
    }

    return percentage_match;
}

/*
  This function finds the KL divergence score between two pdf
  */
double findKLdivergenceScore(vector<double> srcVec,vector<double> dstVec)
{
    double klScore = 0.0;

    for(int i=0;i<srcVec.size();i++)
    {
//        cout << srcVec[i] << "\t" << dstVec[i] << endl;
        if(dstVec[i] > 0 && srcVec[i] > 0)
            klScore += srcVec[i] * log(double(srcVec[i]/dstVec[i]));
        else
            klScore += 0.0;
    }


    return klScore;
}

/*
  This function calculated the distance of each points stored inn the array from cource.
  It retuns a vector containing indics of the k nearest neighbor leaving aside the same index.
  */
void findVectorDistanceFromSource(int src,vector<cv::Point2f> const &array, vector<int> &result,int k)
{
    vector<double> distVector;
    for(int i=0;i<array.size();i++)
    {
        double dist = cv::norm(array[src]-array[i]);
        distVector.push_back(dist);
    }

    vector<int> sortResultVector;
    cv::sortIdx(distVector,sortResultVector,cv::SORT_ASCENDING+cv::SORT_EVERY_ROW);

//    for(int i=0;i<distVector.size();i++)
//        cout << i << "\t" << distVector[i] << "\t" << sortResultVector[i] << endl;

//    exit(0);
    for(int i=0;i<k+1;i++)
    {
        if(sortResultVector[i] != src)
            result.push_back(sortResultVector[i]);
    }

    return;
}
/*
  This function is the extenstion of 1 findVectorDistanceFromSource defined above
  This function calculates kNN of the each feature and if those feature lies below a
  certain threshold. It also consider visual word pair with features itself i.e. self
  word.

  */
void findWordsInARange(int src,vector<cv::Point2f> const &array,vector<int> &result,int k,double neighRadius)
{

//    neighRadius *= 4;
    vector<double> distVector;
    for(int i=0;i<array.size();i++)
    {
        double dist = cv::norm(array[src]-array[i]);
        if(dist < neighRadius)
            distVector.push_back(dist);
        else
            distVector.push_back(1000);
    }

    vector<int> sortResultVector;
    cv::sortIdx(distVector,sortResultVector,cv::SORT_EVERY_ROW+cv::SORT_EVERY_ROW);

    for(int i=0;i<sortResultVector.size();i++)
    {
        if(distVector[sortResultVector[i]] < neighRadius)
            result.push_back(sortResultVector[i]);
    }

    return;

}
/*
  This function is extension of findWordsInRange. Only difference is it does not include
  self word pair ie visual word pair with itself.
  */

void findWordsInARange1(int src,vector<cv::Point2f> const &array,vector<int> &result,int k, double neighRadius)
{

    //    neighRadius *= 2;
        vector<double> distVector;
        for(int i=0;i<array.size();i++)
        {
            double dist = cv::norm(array[src]-array[i]);
            if(dist < neighRadius)
                distVector.push_back(dist);
            else
                distVector.push_back(1000);
        }

        vector<int> sortResultVector;
        cv::sortIdx(distVector,sortResultVector,cv::SORT_EVERY_ROW+cv::SORT_ASCENDING);


        for(int i=0;i<sortResultVector.size();i++)
        {
            if(distVector[sortResultVector[i]] < neighRadius && sortResultVector[i] != src)
                 result.push_back(sortResultVector[i]);
        }

    return;
}

/*
    Predicate function for checking whether the value is non-zero or not in a vector
*/
bool IsZero(int i) {
    return (i != 0);
}

/*
  Normalize a histogram whose values sum to a factor
  */
bool normalizeHist(vector<double> &histVector,double factor)
{
    double sumFactor = 0.0;
    for(int i=0;i<histVector.size();i++)
        sumFactor+=histVector[i];

    for(int i=0;i<histVector.size();i++)
        histVector[i]=double(histVector[i]*factor)/sumFactor;



}

/*
  This function reassign the posterior distribution for the next timestamp.
  */
void reassignPosteriorNew(vector<double> &temp,vector<double> &posterior, int &lastNode)
{
    posterior.resize(temp.size()-1,0.0);
    double value = temp[temp.size()-1];
    for(int i=0;i<posterior.size();i++)
    {
        if(lastNode == 0)
        {
            if(i==lastNode || i==lastNode+1)
            {
                posterior[i] = temp[i] + 0.5 * value;
            }
            else
            {
                posterior[i] = temp[i];
            }
        }
        else
        {
            if(i==lastNode || i== lastNode+1 || i== lastNode-1)
            {
                posterior[i] = temp[i] + 0.33333 * value;
            }
            else
            {
                posterior[i] = temp[i];
            }
        }
    }

    return ;
}

void getLikelihoodTime(vector<double> &hypothesisScore, vector<double> &likelihoodVector, vector<int> &nodeSequence)
{
    vector<double> tempHypthScore(hypothesisScore.begin(),hypothesisScore.end());

    // Setting null likelihood for recently visited nodes(last p images are not considered as loop closures)

    if(nodeSequence.size() > 32)
    {
        for(int i=0;i<hypothesisScore.size();i++)
        {
            for(int j=1;j<=32;j++)
            {
                if(i==nodeSequence[nodeSequence.size()-j])
                {
                    hypothesisScore[i] = 0.0;
                    break;
                }

            }
        }
    }

    // Above scoring is used to calculate the likelihood
    // Calculating mean and deviation of score values of current image with all the existing nodes
    vector<double> meanScore,stdDevScore;
    cv::meanStdDev(hypothesisScore,meanScore,stdDevScore);


    /* Likelihood is defined by Pztlit = { s(zt,zi)-s_stdev/s_mean if s(zt,zi) > s_mean+s_stdev;
                                                1              otherwise

     */

    if(meanScore[0] == 0.0 && stdDevScore[0] == 0.0)
    {
        for(int i=0;i<hypothesisScore.size();i++)
            likelihoodVector.push_back(1.0);

    }
    else
    {
        for(int i=0;i<hypothesisScore.size();i++)
        {
            double condition_value = hypothesisScore[i]-stdDevScore[0]-meanScore[0];
            double P_zi_lit = (double)(hypothesisScore[i]-stdDevScore[0])/(meanScore[0]);
            //        cout << "condition_value \t" << condition_value << endl;
            if(condition_value >= 0)
                likelihoodVector.push_back(P_zi_lit);
            else
                likelihoodVector.push_back(1.0);
        }
    }

    double value = double(meanScore[0])/stdDevScore[0];
    if(isnan(value) || isinf(value))
        likelihoodVector.push_back(1.0);
    else
        likelihoodVector.push_back(1+value);

//    for(int i=0;i<likelihoodVector.size();i++)
//        cout << "Likelihood \t" << i << "\t" << likelihoodVector[i] << endl;

    return;
}

void getPosteriorDistributionTime(vector<double> &prior, vector<double> &likelihood, vector<double> &posterior,
                                  vector<int> &nodeSequence)
{
    posterior.resize(prior.size(),0.0);

    for(int i=0;i<prior.size();i++)
    {
        posterior[i] = prior[i] * likelihood[i];
        if(i==nodeSequence[nodeSequence.size()-1] || i==nodeSequence[nodeSequence.size()-2] ||
                i==nodeSequence[nodeSequence.size()-3] || i== nodeSequence[nodeSequence.size()-4])
        {
            posterior[i] = 0.0;
        }
    }


    double sumValue = cv::sum(posterior)[0];

//    cout << "sum value of posterior before normalisation \t" << sumValue << endl;


    for(int i=0;i<posterior.size();i++)
    {
        posterior[i]/=sumValue;
//        cout << i << "\t" << posterior[i] << endl;
    }

    cout << "sum value of posterior after normalisation \t" << cv::sum(posterior)[0] << endl;

    return;
}

void mygetPriorDistributionNewDashTime(vector<double> &lastPosteriorVector, vector<double> &currentPriorVector, int &lastState, vector<int> &nodeSequence)
{
    currentPriorVector.resize(lastPosteriorVector.size()+1,0.0);

    vector< vector<double> > transitionMatrix;
    vector<double> tempV(currentPriorVector.size(),0.0);
    for(int i=0;i<lastPosteriorVector.size();i++)
        transitionMatrix.push_back(tempV);

//    for(int i=0;i<lastPosteriorVector.size();i++)
//    {
//        if(lastState != -1)
//        {
//        if(i==nodeSequence[nodeSequence.size()-1] || i==nodeSequence[nodeSequence.size()-2] ||
//                i==nodeSequence[nodeSequence.size()-2] || i== nodeSequence[nodeSequence.size()-3])
//            lastPosteriorVector[i] = 0.0;
//        }
//        else
//        {
//            if(i==nodeSequence[nodeSequence.size()-2] || i==nodeSequence[nodeSequence.size()-3] ||
//                    i==nodeSequence[nodeSequence.size()-4] || i== nodeSequence[nodeSequence.size()-5])
//                lastPosteriorVector[i] = 0.0;
//        }
//    }

//    vector<double> probMass(lastPosteriorVector.size(),1.0);

    for(int i=0;i<lastPosteriorVector.size();i++)
    {
        for(int j=0;j<currentPriorVector.size();j++)
        {
            if(i == lastPosteriorVector.size()-1)
            {
                if(lastState == -1)
                {
                    if(j==currentPriorVector.size()-1)
                    {
                        transitionMatrix[i][j] = 0.9;
                    }
                    else
                    {
                        if(j!=nodeSequence[nodeSequence.size()-2] && j!=nodeSequence[nodeSequence.size()-3] &&
                                j!=nodeSequence[nodeSequence.size()-4] && j!= nodeSequence[nodeSequence.size()-5])
                        {
                            if(currentPriorVector.size()-5 != 0)
                                transitionMatrix[i][j] = double(0.1)/(currentPriorVector.size()-5);
                            else
                                transitionMatrix[i][j] = 0.1;
                        }
                        else
                            transitionMatrix[i][j] = 0.0;
                    }
                }
                else
                {
//                    transitionMatrix[i][j] = 0.0;
                    if(j!=nodeSequence[nodeSequence.size()-1] && j!=nodeSequence[nodeSequence.size()-2] &&
                              j!=nodeSequence[nodeSequence.size()-3] && j!= nodeSequence[nodeSequence.size()-4])
                        transitionMatrix[i][j] = double(1.0)/(currentPriorVector.size()-4);
                    else
                        transitionMatrix[i][j] = 0.0;
                }
            }
            else if(i==0)
            {
                if(lastState != -1)
                {
                    if(j==currentPriorVector.size()-1)
                    {
                        transitionMatrix[i][j] = 0.1;
                    }
                    else if(j!=nodeSequence[nodeSequence.size()-1] && j!=nodeSequence[nodeSequence.size()-2] &&
                            j!=nodeSequence[nodeSequence.size()-3] && j!= nodeSequence[nodeSequence.size()-4])
                    {
                        if(j==0 || j==1)
                        {
                            if(j==0)
                            {
                                transitionMatrix[i][j]=0.45;
                            }
                            else
                            {
                                transitionMatrix[i][j] = 0.35;

                            }
                        }
                        else
                        {
                            if(currentPriorVector.size()-7 != 0)
                                transitionMatrix[i][j] = double(0.1)/(currentPriorVector.size()-7);
                            else
                                transitionMatrix[i][j] = 0.1;



                        }

                    }
                    else
                    {
                        transitionMatrix[i][j] = 0.0;

                    }
                }
                else
                {
                    if(j==currentPriorVector.size()-1)
                        transitionMatrix[i][j] = 0.1;
                    else if(j!=nodeSequence[nodeSequence.size()-2] && j!=nodeSequence[nodeSequence.size()-3] &&
                            j!=nodeSequence[nodeSequence.size()-4] && j!= nodeSequence[nodeSequence.size()-5])
                    {
                        if(j==0 || j==1)
                        {
                            if(j==0)
                                transitionMatrix[i][j]=0.45;
                            else
                                transitionMatrix[i][j] = 0.35;
                        }
                        else
                        {
                            if(currentPriorVector.size()-7 != 0)
                            transitionMatrix[i][j] = double(0.1)/(currentPriorVector.size()-7);
                            else
                                transitionMatrix[i][j] = 0.1;

                        }
                    }
                    else
                    {
                        transitionMatrix[i][j] = 0.0;
                    }
                }

            }
            else
            {
                if(lastState != -1)
                {
                    // variable for correct mass allocation
                    double mass = 0.0;

                    if(i == nodeSequence[nodeSequence.size()-1] || i == nodeSequence[nodeSequence.size()-2] ||
                             i == nodeSequence[nodeSequence.size()-3] || i == nodeSequence[nodeSequence.size()-4])
                    {
                            mass = 0.30;
                    }
                    if(i+1 == nodeSequence[nodeSequence.size()-1] || i+1 == nodeSequence[nodeSequence.size()-2] ||
                            i+1 == nodeSequence[nodeSequence.size()-3] || i+1 == nodeSequence[nodeSequence.size()-4])
                    {
                        mass += 0.25;
                    }


                    if(i-1 == nodeSequence[nodeSequence.size()-1] || i-1 == nodeSequence[nodeSequence.size()-2] ||
                            i-1 == nodeSequence[nodeSequence.size()-3] || i-1 == nodeSequence[nodeSequence.size()-4])
                    {
                        mass += 0.25;
                    }

                    if(j==currentPriorVector.size()-1)
                        transitionMatrix[i][j] = 0.1;
                    else if(j!=nodeSequence[nodeSequence.size()-1] && j!=nodeSequence[nodeSequence.size()-2] &&
                            j!=nodeSequence[nodeSequence.size()-3] && j!= nodeSequence[nodeSequence.size()-4])
                    {
                        if(j==i || j==i+1 || j==i-1)
                        {
                            if(j==i)
                                transitionMatrix[i][j]=0.30;
                            else
                                transitionMatrix[i][j] = 0.25;
                        }
                        else
                        {
                            if(mass == 0.0)
                            {
                            if(currentPriorVector.size()-8 != 0)
                            transitionMatrix[i][j] = double(0.1)/(currentPriorVector.size()-8);
                            else
                                transitionMatrix[i][j] = 0.1;
                            }
                            else if(mass == 0.30 || mass == 0.25)
                            {
                                if(currentPriorVector.size()-7 != 0)
                                transitionMatrix[i][j] = double(0.1+mass)/(currentPriorVector.size()-7);
                                else
                                    transitionMatrix[i][j] = 0.1+mass;
                            }
                            else if(mass == 0.55 || mass == 0.50)
                            {
                                if(currentPriorVector.size()-6 != 0)
                                transitionMatrix[i][j] = double(0.1+mass)/(currentPriorVector.size()-6);
                                else
                                    transitionMatrix[i][j] = 0.1+mass;
                            }
                            else
                            {
                                if(currentPriorVector.size()-5 != 0)
                                transitionMatrix[i][j] = double(0.1+mass)/(currentPriorVector.size()-5);
                                else
                                    transitionMatrix[i][j] = 0.1+mass;
                            }


                        }

                    }
                    else
                    {
                        transitionMatrix[i][j] = 0.0;

                    }
                }
                else
                {
//                    if(j==currentPriorVector.size()-1)
//                        transitionMatrix[i][j] = 0.1;

                    // variable for correct mass allocation
//                    double mass = 0.0;

//                    if(i == nodeSequence[nodeSequence.size()-2] || i == nodeSequence[nodeSequence.size()-3] ||
//                             i == nodeSequence[nodeSequence.size()-4] || i == nodeSequence[nodeSequence.size()-5])
//                    {
//                            mass = 0.40;
//                    }
//                    else if(i+1 == nodeSequence[nodeSequence.size()-1] || i+1 == nodeSequence[nodeSequence.size()-2] ||
//                            i+1 == nodeSequence[nodeSequence.size()-3] || i+1 == nodeSequence[nodeSequence.size()-4] ||
//                            i-1 == nodeSequence[nodeSequence.size()-1] || i-1 == nodeSequence[nodeSequence.size()-2] ||
//                            i-1 == nodeSequence[nodeSequence.size()-3] || i-1 == nodeSequence[nodeSequence.size()-4])
//                    {
//                        mass = 0.25;
//                    }
                    // variable for correct mass allocation
                    double mass = 0.0;

                    if(i == nodeSequence[nodeSequence.size()-2] || i == nodeSequence[nodeSequence.size()-3] ||
                             i == nodeSequence[nodeSequence.size()-4] || i == nodeSequence[nodeSequence.size()-5])
                    {
                            mass = 0.40;
                    }
                    if(i+1 == nodeSequence[nodeSequence.size()-2] || i+1 == nodeSequence[nodeSequence.size()-3] ||
                            i+1 == nodeSequence[nodeSequence.size()-4] || i+1 == nodeSequence[nodeSequence.size()-5])
                    {
                        mass += 0.25;
                    }


                    if(i-1 == nodeSequence[nodeSequence.size()-2] || i-1 == nodeSequence[nodeSequence.size()-3] ||
                            i-1 == nodeSequence[nodeSequence.size()-4] || i-1 == nodeSequence[nodeSequence.size()-5])
                    {
                        mass += 0.25;
                    }
                    if(j!=nodeSequence[nodeSequence.size()-2] && j!=nodeSequence[nodeSequence.size()-3] &&
                            j!=nodeSequence[nodeSequence.size()-4] && j!= nodeSequence[nodeSequence.size()-5])
                    {
                        if(j==i || j==i+1 || j==i-1)
                        {
                            if(j==i)
                                transitionMatrix[i][j]=0.40;
                            else
                                transitionMatrix[i][j] = 0.25;
                        }
                        else
                        {
                            if(mass == 0.0)
                            {
                            if(currentPriorVector.size()-7 != 0)
                            transitionMatrix[i][j] = double(0.1)/(currentPriorVector.size()-7);
                            else
                                transitionMatrix[i][j] = 0.1;
                            }
                            else if(mass == 0.40 || mass == 0.25)
                            {
                                if(currentPriorVector.size()-6 != 0)
                                transitionMatrix[i][j] = double(0.1+mass)/(currentPriorVector.size()-6);
                                else
                                    transitionMatrix[i][j] = 0.1+mass;
                            }
                            else if(mass == 0.65 || mass == 0.50)
                            {
                                if(currentPriorVector.size()-5 != 0)
                                transitionMatrix[i][j] = double(0.1+mass)/(currentPriorVector.size()-5);
                                else
                                    transitionMatrix[i][j] = 0.1+mass;
                            }
                            else
                            {
                                if(currentPriorVector.size()-4 != 0)
                                transitionMatrix[i][j] = double(0.1+mass)/(currentPriorVector.size()-4);
                                else
                                    transitionMatrix[i][j] = 0.1+mass;
                            }

                        }
                    }
                    else
                    {
                        transitionMatrix[i][j] = 0.0;
                    }
                }
            }

        }
    }


    for(int i=0;i<currentPriorVector.size();i++)
    {
        for(int j=0;j<lastPosteriorVector.size();j++)
        {
            currentPriorVector[i] += (transitionMatrix[j][i] * lastPosteriorVector[j]);
        }
    }

    cout << "sum \t" << cv::sum(currentPriorVector)[0] << endl;
    if(isinf(cv::sum(currentPriorVector)[0]) || isnan(cv::sum(currentPriorVector)[0]) || cv::sum(currentPriorVector)[0] > 1.0001
             || cv::sum(currentPriorVector)[0] < 0.99)
    {

//        for(int i=0;i<lastPosteriorVector.size();i++)
//            cout << i << "\t" << lastPosteriorVector[i] << endl;

//        for(int i=0;i<lastPosteriorVector.size();i++)
//            for(int j=0;j<currentPriorVector.size();j++)
//            {
//                cout << i << "\t" << j << "\t" << transitionMatrix[i][j] << endl;
//            }
//        for(int i=0;i<currentPriorVector.size();i++)
//            cout << i << "\t" << currentPriorVector[i] << endl;

//        for(int i=0;i<lastPosteriorVector.size();i++)
//            cout << i << "\t" << cv::sum(transitionMatrix[i])[0] << endl;

//        exit(0);
    }
    return;


}

/*
 * Function to find the loop closure candidate by finding the node having maximum probability
 * */
void getLoopClosureCandidateTime(vector<double> const &posterior,int &possibleLoopclosureNode, double &probabilityValue
                             ,vector<int> &nodeSequence)
{
    probabilityValue = 0.0;
    possibleLoopclosureNode = 0;


    // This -1 is because last posterior is for no loop closure
    for(int i=0;i<posterior.size()-1;i++)
    {
        if(posterior[i] > probabilityValue)
        {
            probabilityValue = posterior[i];
            possibleLoopclosureNode = i;
        }
    }

    // Check whether new node creation probaility is higher
    if(probabilityValue < posterior[posterior.size()-1])
    {
        possibleLoopclosureNode = posterior.size()-1;
        probabilityValue = posterior[posterior.size()-1];
    }

    return;
}

/*
 * Function to find the K candidates having maximum probability
 */

void getKcandidatesForVerification(vector<double> const &posterior, vector<int> &candidateList, vector<double> &candidateProbability
                                   ,int numberOfCandidates)
{
    vector<int> indexVector;
    cv::sortIdx(posterior,indexVector,cv::SORT_DESCENDING+cv::SORT_EVERY_ROW);

    for(int i=0;i<numberOfCandidates;i++)
    {
        candidateList.push_back(indexVector[i]);
        candidateProbability.push_back(posterior[indexVector[i]]);
    }

    return;
}

/*
 * Function to find the K candidates having maximum probability with some constraints
 */

void getKcandidatesForVerification1(vector<double> const &posterior, vector<int> &candidateList, vector<double> &candidateProbability
                                   ,int numberOfCandidates, vector<int> &nodeSequence)
{
    vector<int> indexVector;
    cv::sortIdx(posterior,indexVector,cv::SORT_DESCENDING+cv::SORT_EVERY_ROW);



    for(int i=0;i<numberOfCandidates;i++)
    {
        candidateList.push_back(indexVector[i]);
        candidateProbability.push_back(posterior[indexVector[i]]);
    }

    return;
}






/*
 * Function to find the loop closure candidate by finding the node having maximum sum of probability in its neighborhood
 */

void getLoopClosureCandidateTimeInNeighbors(vector<double> const &posterior,int &possibleLoopclosureNode, double &probabilityValue
                                 ,vector<int> &nodeSequence)
{
    probabilityValue = 0.0;
    double tempSumValue;
    possibleLoopclosureNode = 0;
//#if(NEW_COLLEGE || CITY_CENTER)
    for(int i=0;i<posterior.size()-1;i++)
    {
        double sum = 0.0;
        if(i==0 || i==1)
        {
            sum = posterior[i] + posterior[i+2];
        }
        else if( i == (posterior.size()-2) || i == (posterior.size()-3))
        {
            sum = posterior[i] + posterior[i-2];
        }
        else
        {
            sum = posterior[i-2] + posterior[i] + posterior[i+2];
        }

        if(sum > tempSumValue)
        {
            tempSumValue = sum;
            probabilityValue = posterior[i];
            possibleLoopclosureNode = i;
        }

    }
#if(LIP_OUTDOOR)
    for(int i=0;i<posterior.size()-1;i++)
    {
        double sum = 0.0;
        if(i==0)
            sum = posterior[i] + posterior[i+1];
        else if(i==posterior.size()-2)
            sum = posterior[i] + posterior[i-1];
        else
            sum = posterior[i-1] + posterior[i] + posterior[i+1];

        if(sum > probabilityValue)
        {
            probabilityValue = sum;
            possibleLoopclosureNode = i;
        }

    }
#endif

    // Check whether new node probability is higher
    if(probabilityValue < posterior[posterior.size()-1])
    {
        possibleLoopclosureNode = posterior.size()-1;
        probabilityValue = posterior[posterior.size()-1];
    }

    return;

}

/*
 * Function to find a fixed number of descriptors which has maximum dominance in the image
*/
void findFixedFeatures(vector<cv::KeyPoint> &keyp, vector<cv::KeyPoint> &keyOut, int descriptorLimit)
{

    if(keyp.size() < descriptorLimit)
    {
        keyOut.assign(keyp.begin(),keyp.end());
        return;
    }

    vector<int> inputArray,outArray;
    for(int i=0;i<keyp.size();i++)
        inputArray.push_back(keyp[i].response);

    cv::sortIdx(inputArray,outArray,CV_SORT_DESCENDING+CV_SORT_EVERY_ROW);

    for(int i=0;i<descriptorLimit;i++)
        keyOut.push_back(keyp[outArray[i]]);

//    for(int i=0;i<keyOut.size();i++)
//        cout << keyOut[i].response << endl;
    return;
}

/*
 * Function to find a fixed number of descriptors which has maximum dominance in the image
*/
void findFixedFeatures(vector<cv::KeyPoint> &keyp, vector<cv::KeyPoint> &keyOut,
                       cv::Mat &descIn,cv::Mat &descOut, int descriptorLimit)
{

    if(keyp.size() < descriptorLimit)
    {
        keyOut.assign(keyp.begin(),keyp.end());
        descOut = descIn.clone();

        return;
    }

    vector<int> inputArray,outArray;
    for(int i=0;i<keyp.size();i++)
        inputArray.push_back(keyp[i].response);

    cv::sortIdx(inputArray,outArray,CV_SORT_DESCENDING+CV_SORT_EVERY_ROW);

    for(int i=0;i<descriptorLimit;i++)
    {
        keyOut.push_back(keyp[outArray[i]]);
        descOut.push_back(descIn.row(outArray[i]));
    }



//    for(int i=0;i<keyOut.size();i++)
//        cout << keyOut[i].response << endl;
    return;
}

/*
 * This function is to find the predicted probability at timestamp t. The probabilites are defined based on
 * the transition model. This function is specifically assuming some of the observation in the SLAM problem.
 */
void predictedProbabilityWithTransitionModel(vector<double> &lastPosteriorVector, vector<double> &currentPriorVector,
                                             int &lastState, vector<int> &nodeSequence, map<int,vector<int> > &neighborMap)
{
    currentPriorVector.resize(lastPosteriorVector.size()+1,0.0);

    vector< vector<double> > transitionMatrix;
    vector<double> tempV(currentPriorVector.size(),0.0);
    for(int i=0;i<lastPosteriorVector.size();i++)
        transitionMatrix.push_back(tempV);

    // Gaussain probability values
    vector<double> gaussianProbValues;
    gaussianProbValues.push_back(0.03);
    gaussianProbValues.push_back(0.05);
    gaussianProbValues.push_back(0.08);
    gaussianProbValues.push_back(0.18);
    gaussianProbValues.push_back(0.18);
    gaussianProbValues.push_back(0.08);
    gaussianProbValues.push_back(0.05);
    gaussianProbValues.push_back(0.03);


    // Construct a transition matrix based on the transition model


    /*
     * Section for the last row of the transition matrix
     */

    // Case when last node is a new node
    if(lastState == -1)
    {
        // Case : When current node is a new node
        transitionMatrix[lastPosteriorVector.size()-1][currentPriorVector.size()-1] = 0.9;

        // Loop for the last row in the transition matrix except new node
        for(int i=0;i<currentPriorVector.size()-1;i++)
        {
            transitionMatrix[lastPosteriorVector.size()-1][i] = (double)(0.1)/(currentPriorVector.size()-1);
        }



    }
    else
    {
        // Case when transition is from loop closure to new node

        //Setting transition probability to be 0.1 for a transition from loop closure to new node
        transitionMatrix[lastPosteriorVector.size()-1][currentPriorVector.size()-1] = 0.1;

        transitionMatrix[lastPosteriorVector.size()-1][currentPriorVector.size()-2] = gaussianProbValues[3]+0.14;
        transitionMatrix[lastPosteriorVector.size()-1][currentPriorVector.size()-3] = gaussianProbValues[2]+0.14;
        transitionMatrix[lastPosteriorVector.size()-1][currentPriorVector.size()-4] = gaussianProbValues[1]+0.14;
        transitionMatrix[lastPosteriorVector.size()-1][currentPriorVector.size()-5] = gaussianProbValues[0]+0.14;


    }


    /*
     * Section for writing the other rows of the matrix except last one
     */

    // Case when last node was a loop closure
    // Inserting rows elements according to 3rd and 4th transition model rules
    for(int i=0;i<lastPosteriorVector.size()-1;i++)
    {

        // Initialisng al the transition probaility to be zero
        for(int j=0;j<currentPriorVector.size();j++)
        {
            transitionMatrix[i][j] = 0.0;
        }

        //Setting transition probability to be 0.1 for a transition from loop closure to new node
        transitionMatrix[i][currentPriorVector.size()-1] = 0.1;



        // Assigning probabilites to neighbors
        vector<int> neighborList(int(NEIGHBOR_THRESHOLD),-1);
        std::map<int, vector<int> >::iterator p = neighborMap.find(i);
        double probabilityMass = 0.9;
        int neighborCount = 0;


        for(int k=0;k<int(NEIGHBOR_THRESHOLD);k++)
        {
            neighborList[k] = p->second[k];
//            cout << "Neighbor List " << "\t" << i << "\t" << k << "\t" << neighborList[k] << "\t" << p->second[k] << endl;

            if(neighborList[k] >=0)
            {
//                cout << i << "\t" << "Neighbor List \t" << k << "\t" << neighborList[k] << endl;
                transitionMatrix[i][neighborList[k]] = gaussianProbValues[k];
                probabilityMass = probabilityMass-gaussianProbValues[k];
                neighborCount++;
            }
        }

        // Assigning probability to transition to same node

        transitionMatrix[i][i] = 0.9-cv::sum(gaussianProbValues)[0];
        probabilityMass = probabilityMass - transitionMatrix[i][i];

        cout << "Remaining proability mass \t" << probabilityMass << endl;

        // If probability mass is remaining because of some neighbors not present, remaining probability mass is assigned
        // unifroamlly to each neighbor
        if(probabilityMass > 0.0)
        {
            double value = double(probabilityMass)/neighborCount;
            for(int k=0;k<int(NEIGHBOR_THRESHOLD);k++)
            {
                if(neighborList[k] >=0)
                {
                    transitionMatrix[i][neighborList[k]] += value;
                    probabilityMass = probabilityMass-value;
                }
            }
        }




        cout << "After Remaining proability mass \t" << probabilityMass << endl;

//        cout << "HELOOO\t" << cv::sum(transitionMatrix[i])[0] << "\t" << transitionMatrix[i][i] << endl;

    }



    for(int i=0;i<lastPosteriorVector.size();i++)
    {
        double sum=0.0;
        for(int j=0;j<currentPriorVector.size();j++)
        {
            sum += transitionMatrix[i][j];
        }
        cout << i << "\t" << "row sum \t" << sum << endl;
    }


    // Multiplying the transition matrix with t-1 posterior to get prior at t timestamp

    for(int i=0;i<currentPriorVector.size();i++)
    {
        for(int j=0;j<lastPosteriorVector.size();j++)
        {
            currentPriorVector[i] += (transitionMatrix[j][i] * lastPosteriorVector[j]);
        }

//        cout << currentPriorVector[i] << "\t";
    }

    cout << "sum \t" << cv::sum(currentPriorVector)[0] << endl;
    if(isinf(cv::sum(currentPriorVector)[0]) || isnan(cv::sum(currentPriorVector)[0]) || cv::sum(currentPriorVector)[0] > 1.0001
             || cv::sum(currentPriorVector)[0] < 0.99)
    {
       cout << "Probability sum not equal to 1, Please check the function" << endl;
    }

    return ;
}

/*
 * This function is to keep track of the neighbors for each node so that whenever a loop closure is detected
 * the prediciton probabilities for the next time stamp can be distributed among the neigbors properly.
 */
void addNeighborsToNode(vector<int> &nodeSequence,map<int,vector<int> > &neighborInformation)
{
    /*
     * This function should be called only when a new node is created
     */

    // initialising neighborhood map where it stores information about neigboring node for each node
    neighborInformation.insert(std::pair<int, vector<int> >(nodeSequence[nodeSequence.size()-1],vector<int>(8,-1)));

    // Accessing node sequence vector to extract out information of neighborhood nodes for each node

    int loopCount = 0;
    for(int i=nodeSequence.size()-1;i>nodeSequence.size()-6,i>=0;i--,loopCount++)
    {
        int sourceId = nodeSequence[i];
        std::map<int, vector<int> >::iterator p = neighborInformation.find(sourceId);


        switch(loopCount)
        {
        case 0:
            if(nodeSequence.size() >= 4)
            {
                p->second[0] = nodeSequence[i-4];
                p->second[1] = nodeSequence[i-3];
                p->second[2] = nodeSequence[i-2];
                p->second[3] = nodeSequence[i-1];
            }
            else if(nodeSequence.size() >= 3)
            {
                p->second[0] = nodeSequence[i-3];
                p->second[1] = nodeSequence[i-2];
                p->second[2] = nodeSequence[i-1];
            }
            else if(nodeSequence.size() >= 2)
            {
                p->second[0] = nodeSequence[i-2];
                p->second[1] = nodeSequence[i-1];
            }
            else if(nodeSequence.size() >= 1)
            {
                p->second[0] = nodeSequence[i-1];

            }
            cout << "Source id \t" << sourceId << "\t" << p->second[0] << "\t" <<  p->second[1] << "\t" << p->second[2] << "\t" << p->second[3] << endl;
            break;
        case 1 :
            p->second[4] = nodeSequence[i+1];
//            cout << "Source id \t" << p->second[4] << endl;
            break;
        case 2:
            p->second[5] = nodeSequence[i+2];
//            cout << "Source id \t" << p->second[5] << endl;

            break;
        case 3:
            p->second[6] = nodeSequence[i+3];
//            cout << "Source id \t" << p->second[6] << endl;

            break;
        case 4:
            p->second[7] = nodeSequence[i+4];
//            cout << "Source id \t" << p->second[7] << endl;

            break;
        default :
//            cout << "No matching value .. exiting switch" << endl;
            break;
        }

//        cout << "loopcount \t" << loopCount << endl;

    }



   return ;
}

#endif // AUGTOPOMAP_H
