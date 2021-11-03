#include "feature_tracker.h"

int FeatureTracker::n_id = 0;

bool inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
}

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}


FeatureTracker::FeatureTracker()
{
}

void FeatureTracker::setMask()
{
    if(FISHEYE)
        mask = fisheye_mask.clone();
    else
        mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));
    
    // prefer to keep features that are tracked for long time
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

    for (unsigned int i = 0; i < forw_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(forw_pts[i], ids[i])));

    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;
         });


    forw_pts.clear();
    ids.clear();
    track_cnt.clear();
    
    for (auto &it : cnt_pts_id)
    {   
        if (mask.at<uchar>(it.second.first) == 255)
        {
            forw_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
        }
    }
    // cout<<"forw_pts.size() = "<<forw_pts.size()<<endl;

}

// void FeatureTracker::setMask()
// {
//     if(FISHEYE)
//         mask = fisheye_mask.clone();
//     else
//         mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));
    

//     // prefer to keep features that are tracked for long time
//     vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

//     for (unsigned int i = 0; i < forw_pts.size(); i++)
//         cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(forw_pts[i], ids[i])));

//     sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
//          {
//             return a.first > b.first;
//          });

//     forw_pts.clear();
//     ids.clear();
//     track_cnt.clear();

//     for (auto &it : cnt_pts_id)
//     {
//         if (mask.at<uchar>(it.second.first) == 255)
//         {
//             forw_pts.push_back(it.second.first);
//             ids.push_back(it.second.second);
//             track_cnt.push_back(it.first);
//             cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
//         }
//     }
   
// }

void FeatureTracker::addPoints()
{
    for (auto &p : n_pts)
    {
        forw_pts.push_back(p);
        ids.push_back(-1);
        track_cnt.push_back(1);
    }
}

void FeatureTracker::readImage(const cv::Mat &_img, double _cur_time)
{
    cv::Mat img;
    TicToc t_r;
    cur_time = _cur_time;

    if (EQUALIZE)
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        TicToc t_c;
        clahe->apply(_img, img);
        ROS_DEBUG("CLAHE costs: %fms", t_c.toc());
    }
    else
        img = _img;

    if (forw_img.empty())
    {
        prev_img = cur_img = forw_img = img;
    }
    else
    {
        forw_img = img;
    }

    forw_pts.clear();

    if (cur_pts.size() > 0)
    {
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;
        cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);

        for (int i = 0; i < int(forw_pts.size()); i++)
            if (status[i] && !inBorder(forw_pts[i]))
                status[i] = 0;
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(ids, status);
        reduceVector(cur_un_pts, status);
        reduceVector(track_cnt, status);
        feture_tracker_cost_ += t_o.toc();
        std::cout << " feture tracker cost : " << feture_tracker_cost_ << " ms" << std::endl;
        ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
    }

    for (auto &n : track_cnt)
        n++;

    if (PUB_THIS_FRAME)
    {
        rejectWithF();
        
        ROS_DEBUG("set mask begins");
        TicToc t_m;
        setMask();
        ROS_DEBUG("set mask costs %fms", t_m.toc());

        ROS_DEBUG("detect feature begins");
        TicToc t_t;
        int n_max_cnt = MAX_CNT - static_cast<int>(forw_pts.size());
        cout<<"n_max_cnt = "<< n_max_cnt<<endl;
        if (n_max_cnt > 0)
        {
            if(mask.empty())
                cout << "mask is empty " << endl;
            if (mask.type() != CV_8UC1)
                cout << "mask type wrong " << endl;
            if (mask.size() != forw_img.size())
                cout << "wrong size " << endl;
            cv::goodFeaturesToTrack(forw_img, n_pts, MAX_CNT - forw_pts.size(), 0.01, MIN_DIST, mask);
        }
        else
            n_pts.clear();
        feture_detect_cost_ += t_t.toc();
        std::cout << " feture detect cost : " << feture_detect_cost_ << " ms" << std::endl;
        ROS_DEBUG("detect feature costs: %fms", t_t.toc());

        ROS_DEBUG("add feature begins");
        TicToc t_a;
        addPoints();
        ROS_DEBUG("selectFeature costs: %fms", t_a.toc());
    }
    // cout<<"cur_pts size = "<<cur_pts.size()<<endl;
    prev_img = cur_img;
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;
    cur_img = forw_img;
    cur_pts = forw_pts;
    undistortedPoints();
    prev_time = cur_time;
}


void FeatureTracker::readImage_mask(const cv::Mat &_img, const cv::Mat& _mask_img, double _cur_time)
{
    cv::Mat img;
    
    TicToc t_r;
    cur_time = _cur_time;

    if (EQUALIZE)
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        TicToc t_c;
        clahe->apply(_img, img);
        mask_img = _mask_img;
        ROS_DEBUG("CLAHE costs: %fms", t_c.toc());
    }
    else
    {
        img = _img;
        mask_img = _mask_img;
    }

    if (forw_img.empty())
    {
        prev_img = cur_img = forw_img = img;
    }
    else
    {
        forw_img = img;
    }

    forw_pts.clear();

    if (cur_pts.size() > 0)
    {
        TicToc t_o;
        vector<uchar> status;
        vector<uchar> status_mask;
        vector<float> err;
        cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);

        // for (int i = 0; i < int(forw_pts.size()); i++)
        //     if (status[i] && !inBorder(forw_pts[i]))
        //         status[i] = 0;

        status_mask = status;
        cur_pts_mask = cur_pts;
        forw_pts_mask = forw_pts;

        for (int i = 0; i < int(forw_pts.size()); i++)
        {
            if (status[i] && !inBorder(forw_pts[i]))  // 剔除边界处的
            {
                status[i] = 0;
                status_mask[i] = 0;
            }

            if(status[i])   // 剔除所有被mask包含的点
            {
                int col = cvRound(forw_pts[i].x);
                int row = cvRound(forw_pts[i].y);

                if(col > 5 && row > 5 && col < ROW-5  && col < COL-5)
                {
                    // cv::Mat访问是 .at（列,行） => (x,y)
                    float mask_piexl_up = mask_img.at<uchar>(col,row-5);  
                    float mask_piexl_down = mask_img.at<uchar>(col,row+5);
                    float mask_piexl_left = mask_img.at<uchar>(col-5,row);
                    float mask_piexl_right = mask_img.at<uchar>(col+5,row);

                    // 剔除掉隐藏的mask动态点
                    if(mask_piexl_up < 1 || mask_piexl_down < 1 || mask_piexl_left < 1 || mask_piexl_right < 1)
                    {
                        status_mask[i] = 0;
                        // status[i] = 0;
                        // cout<<"mask_piexl_up = "<< mask_piexl_up<< "mask_piexl_down = "<< mask_piexl_down<< "mask_piexl_left = "<< mask_piexl_left<<"mask_piexl_right = "<< mask_piexl_right<<endl;
                    }
                }
            }
        }

        reduceVector(cur_pts_mask, status_mask);
        reduceVector(forw_pts_mask, status_mask);

        // reduceVector(prev_pts, status_mask);
        // reduceVector(cur_pts, status_mask);
        // reduceVector(forw_pts, status_mask);
        // reduceVector(ids, status_mask);
        // reduceVector(cur_un_pts, status_mask);
        // reduceVector(track_cnt, status_mask);

        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(ids, status);
        reduceVector(cur_un_pts, status);
        reduceVector(track_cnt, status);

        feture_tracker_cost_ += t_o.toc();
        // std::cout << " feture tracker cost : " << feture_tracker_cost_ << " ms" << std::endl;
        ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
    }

    for (auto &n : track_cnt)
        n++;

    if (PUB_THIS_FRAME)
    {
        // rejectWithF();
        rejectWithF_mask();

        // //add by gf
        // cv::Mat tmp_img;
        // for (int i = 0; i < NUM_OF_CAM; i++)
        // {
        //     tmp_img = forw_img.rowRange(i * ROW, (i + 1) * ROW);
        //     cv::cvtColor(forw_img, tmp_img, CV_GRAY2RGB);

        //     for (unsigned int j = 0; j < forw_pts.size(); j++)
        //     {
        //         double len = std::min(1.0, 1.0 * track_cnt[j] / WINDOW_SIZE);
        //         cv::circle(tmp_img, forw_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);  // 越红说明跟踪的越长
        //     }
        // }
        // cv::imshow("vis", tmp_img);
        // cv::waitKey(1);
        // //

        ROS_DEBUG("set mask begins");
        TicToc t_m;
        setMask();
        ROS_DEBUG("set mask costs %fms", t_m.toc());

        ROS_DEBUG("detect feature begins");
        TicToc t_t;
        int n_max_cnt = MAX_CNT - static_cast<int>(forw_pts.size());
        cout<<" n_max_cnt = "<<n_max_cnt<<endl;
        if (n_max_cnt > 0)
        {
            if(mask.empty())
                cout << "mask is empty " << endl;
            if (mask.type() != CV_8UC1)
                cout << "mask type wrong " << endl;
            if (mask.size() != forw_img.size())
                cout << "wrong size " << endl;
            
            cv::Mat temp_mask;
            mask_img.copyTo(temp_mask, mask);  // 使用减法操作，将图像的mask显示出来，其中mask是得到的结果（输出）
            // cv::imshow("temp_mask", temp_mask);
            // cv::waitKey(1);
            cv::goodFeaturesToTrack(forw_img, n_pts, MAX_CNT - forw_pts.size(), 0.01, MIN_DIST, temp_mask);
            // cv::goodFeaturesToTrack(forw_img, n_pts, MAX_CNT - forw_pts.size(), 0.01, MIN_DIST, mask);
        }
        else
            n_pts.clear();
        feture_detect_cost_ += t_t.toc();
        // std::cout << " feture detect cost : " << feture_detect_cost_ << " ms" << std::endl;
        ROS_DEBUG("detect feature costs: %fms", t_t.toc());

        ROS_DEBUG("add feature begins");
        TicToc t_a;
        addPoints();
        ROS_DEBUG("selectFeature costs: %fms", t_a.toc());
    }

    // cout<<"cur_pts size = "<<cur_pts.size()<<endl;
    prev_img = cur_img;
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;
    cur_img = forw_img;
    cur_pts = forw_pts;
    undistortedPoints();
    prev_time = cur_time;
}

void FeatureTracker::rejectWithF()
{
    if (forw_pts.size() >= 8)
    {
        ROS_DEBUG("FM ransac begins");
        TicToc t_f;
        vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_forw_pts(forw_pts.size());
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            Eigen::Vector3d tmp_p;
            m_camera->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;
        cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        int size_a = cur_pts.size();

        // cout<<"cur_pts.size() : "<<size_a<<endl;
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(cur_un_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);

        // cout<<"cur_pts.size() : "<<cur_pts.size()<<endl;
        ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, forw_pts.size(), 1.0 * forw_pts.size() / size_a);
        ROS_DEBUG("FM ransac costs: %fms", t_f.toc());
    }
}


void FeatureTracker::rejectWithF_mask()
{
    if (forw_pts_mask.size() >= 8)
    {
        ROS_DEBUG("FM ransac begins");
        TicToc t_f;

        // 由不包含动态点计算的F
        vector<cv::Point2f> un_cur_pts_mask(cur_pts_mask.size()), un_forw_pts_mask(forw_pts_mask.size());
        for (unsigned int i = 0; i < cur_pts_mask.size(); i++)
        {
            Eigen::Vector3d tmp_p;
            m_camera->liftProjective(Eigen::Vector2d(cur_pts_mask[i].x, cur_pts_mask[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_cur_pts_mask[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera->liftProjective(Eigen::Vector2d(forw_pts_mask[i].x, forw_pts_mask[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_forw_pts_mask[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;
        cv::Mat Fundamental_mask(3,3,CV_64F);
        // Fundamental_mask = cv::findFundamentalMat(un_cur_pts_mask, un_forw_pts_mask, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        Fundamental_mask = cv::findFundamentalMat(un_cur_pts_mask, un_forw_pts_mask, cv::FM_RANSAC, 0.1, 0.99, status);
        
        // 利用F去除动态点
        vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_forw_pts(forw_pts.size());
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            Eigen::Vector3d tmp_p;
            m_camera->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());


            m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }
        
        int size_a = cur_pts.size();
        vector<uchar> status_mask;

        // cout<<"cur_pts.size() :"<< cur_pts.size()<<endl;

        for(int i = 0; i < size_a; i++)
        {
            status_mask.push_back(1);

            // Circle(pre_frame, un_cur_pts[i], 6, Scalar(255, 255, 0), 3);
            double A = Fundamental_mask.at<double>(0, 0)*un_cur_pts[i].x + Fundamental_mask.at<double>(0, 1)*un_cur_pts[i].y + Fundamental_mask.at<double>(0, 2);
            double B = Fundamental_mask.at<double>(1, 0)*un_cur_pts[i].x + Fundamental_mask.at<double>(1, 1)*un_cur_pts[i].y + Fundamental_mask.at<double>(1, 2);
            double C = Fundamental_mask.at<double>(2, 0)*un_cur_pts[i].x + Fundamental_mask.at<double>(2, 1)*un_cur_pts[i].y + Fundamental_mask.at<double>(2, 2);
            double dd = fabs(A*un_forw_pts[i].x + B*un_forw_pts[i].y + C) / sqrt(A*A + B*B); //Epipolar constraints

            // cout<< dd<<endl;

            if(dd > 1.0) status_mask[i] = 0;
        }

        reduceVector(prev_pts, status_mask);
        reduceVector(cur_pts, status_mask);
        reduceVector(forw_pts, status_mask);
        reduceVector(cur_un_pts, status_mask);
        reduceVector(ids, status_mask);
        reduceVector(track_cnt, status_mask);

        // cout<<"cur_pts.size() :"<< cur_pts.size()<<endl;

        ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, forw_pts.size(), 1.0 * forw_pts.size() / size_a);
        ROS_DEBUG("FM ransac costs: %f ms", t_f.toc());
    }
}

bool FeatureTracker::updateID(unsigned int i)
{
    if (i < ids.size())
    {
        if (ids[i] == -1)
            ids[i] = n_id++;
        return true;
    }
    else
        return false;
}

void FeatureTracker::readIntrinsicParameter(const string &calib_file)
{
    ROS_INFO("reading paramerter of camera %s", calib_file.c_str());
    m_camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
}

void FeatureTracker::showUndistortion(const string &name)
{
    cv::Mat undistortedImg(ROW + 600, COL + 600, CV_8UC1, cv::Scalar(0));
    vector<Eigen::Vector2d> distortedp, undistortedp;
    for (int i = 0; i < COL; i++)
        for (int j = 0; j < ROW; j++)
        {
            Eigen::Vector2d a(i, j);
            Eigen::Vector3d b;
            m_camera->liftProjective(a, b);
            distortedp.push_back(a);
            undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
            //printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
        }
    for (int i = 0; i < int(undistortedp.size()); i++)
    {
        cv::Mat pp(3, 1, CV_32FC1);
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + COL / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + ROW / 2;
        pp.at<float>(2, 0) = 1.0;
        //cout << trackerData[0].K << endl;
        //printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
        //printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
        if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < ROW + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < COL + 600)
        {
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
        }
        else
        {
            //ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x, pp.at<float>(1, 0), pp.at<float>(0, 0));
        }
    }
    cv::imshow(name, undistortedImg);
    cv::waitKey(0);
}

void FeatureTracker::undistortedPoints()
{
    cur_un_pts.clear();
    cur_un_pts_map.clear();
    //cv::undistortPoints(cur_pts, un_pts, K, cv::Mat());
    for (unsigned int i = 0; i < cur_pts.size(); i++)
    {
        Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
        Eigen::Vector3d b;
        m_camera->liftProjective(a, b);
        cur_un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
        cur_un_pts_map.insert(make_pair(ids[i], cv::Point2f(b.x() / b.z(), b.y() / b.z())));
        //printf("cur pts id %d %f %f", ids[i], cur_un_pts[i].x, cur_un_pts[i].y);
    }
    // caculate points velocity
    if (!prev_un_pts_map.empty())
    {
        double dt = cur_time - prev_time;
        pts_velocity.clear();
        for (unsigned int i = 0; i < cur_un_pts.size(); i++)
        {
            if (ids[i] != -1)
            {
                std::map<int, cv::Point2f>::iterator it;
                it = prev_un_pts_map.find(ids[i]);
                if (it != prev_un_pts_map.end())
                {
                    double v_x = (cur_un_pts[i].x - it->second.x) / dt;
                    double v_y = (cur_un_pts[i].y - it->second.y) / dt;
                    pts_velocity.push_back(cv::Point2f(v_x, v_y));
                }
                else
                    pts_velocity.push_back(cv::Point2f(0, 0));
            }
            else
            {
                pts_velocity.push_back(cv::Point2f(0, 0));
            }
        }
    }
    else
    {
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }
    prev_un_pts_map = cur_un_pts_map;
}
