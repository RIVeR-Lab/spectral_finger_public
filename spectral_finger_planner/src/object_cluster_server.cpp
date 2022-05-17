#include <ros/package.h>
#include <point_cloud_proc/point_cloud_proc.h>


class ObjectClustering{
    public:
        explicit ObjectClustering(ros::NodeHandle n);

        bool clusterserver(
            point_cloud_proc::TabletopClustering::Request& req,
            point_cloud_proc::TabletopClustering::Response& res
        );

    private:
        ros::NodeHandle nh;
        PointCloudProc* pcp_;
};

ObjectClustering::ObjectClustering(ros::NodeHandle n) : nh(n){
    std::string pkg_path = ros::package::getPath("spectral_finger_planner");
    std::string pcp_config = pkg_path + "/config/table.yaml";

    pcp_ = new PointCloudProc(nh, true, pcp_config);
    ros::ServiceServer s_cluster = nh.advertiseService(
        "/object_clustering/cluster_objects", &ObjectClustering::clusterserver, this);
    ros::waitForShutdown();
}

bool ObjectClustering::clusterserver(
            point_cloud_proc::TabletopClustering::Request& req,
            point_cloud_proc::TabletopClustering::Response& res){
    std::vector<point_cloud_proc::Object> tabletop_objects;                
    bool tabletop_clustered = pcp_->clusterObjects(tabletop_objects);
    res.success = tabletop_clustered;
    res.objects = tabletop_objects;
    return true;

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "object_clustering");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ObjectClustering server(nh);

}