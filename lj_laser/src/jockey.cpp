#include <lj_laser/jockey.h>

namespace lama {
namespace lj_laser {

Jockey::Jockey(std::string name, const double frontier_width, const double max_frontier_angle) :
  lama::LocalizingJockey(name),
  data_received_(false),
  scan_reception_time_(ros::Time(0)),
  laser_interface_name_(name + "_laser_descriptor"),
  crossing_interface_name_(name + "_crossing_descriptor"),
  dissimilarity_server_name_("dissimilarity_server"),
  crossing_detector_(frontier_width, max_frontier_angle)
{
  private_nh_.getParam("laser_interface_name", laser_interface_name_);
  private_nh_.getParam("crossing_interface_name", crossing_interface_name_);
  private_nh_.getParam("dissimilarity_server_name", dissimilarity_server_name_);

  initMapLaserInterface();
  initMapCrossingInterface();

  // Initialize the client for the dissimilarity server.
  dissimilarity_server_ = nh_.serviceClient<polygon_matcher::PolygonDissimilarity>(dissimilarity_server_name_);
}

/* Create the getter and setter services for LaserScan descriptors.
 */
void Jockey::initMapLaserInterface()
{
  ros::ServiceClient client = nh_.serviceClient<lama_interfaces::AddInterface>("interface_factory");
  client.waitForExistence();
  lama_interfaces::AddInterface srv;
  srv.request.interface_name = laser_interface_name_;
  srv.request.interface_type = lama_interfaces::AddInterfaceRequest::CLEARTEXT;
  srv.request.get_service_message = "lama_interfaces/GetVectorLaserScan";
  srv.request.set_service_message = "lama_interfaces/SetVectorLaserScan";
  if (!client.call(srv))
  {
    ROS_ERROR("Failed to create the Lama interface %s", laser_interface_name_.c_str());
    return;
  }
  // Initialize the clients for the LaserScan getter and setter services (interface to map).
  laser_descriptor_getter_ = nh_.serviceClient<lama_interfaces::GetVectorLaserScan>(srv.response.get_service_name);
  laser_descriptor_getter_.waitForExistence();
  laser_descriptor_setter_ = nh_.serviceClient<lama_interfaces::SetVectorLaserScan>(srv.response.set_service_name);
  laser_descriptor_setter_.waitForExistence();
}

/* Create the setter services for Crossing descriptors.
 */
void Jockey::initMapCrossingInterface()
{
  ros::ServiceClient client = nh_.serviceClient<lama_interfaces::AddInterface>("interface_factory");
  client.waitForExistence();
  lama_interfaces::AddInterface srv;
  srv.request.interface_name = crossing_interface_name_;
  srv.request.interface_type = lama_interfaces::AddInterfaceRequest::CLEARTEXT;
  srv.request.get_service_message = "lama_msgs/GetCrossing";
  srv.request.set_service_message = "lama_msgs/SetCrossing";
  if (!client.call(srv))
  {
    ROS_ERROR("Failed to create the Lama interface %s", crossing_interface_name_.c_str());
  }
  // Initialize the client for the Crossing setter services (interface to map).
  crossing_descriptor_setter_ = nh_.serviceClient<lama_msgs::SetCrossing>(srv.response.set_service_name);
  crossing_descriptor_setter_.waitForExistence();
}

/* Start the subscriber, wait for a LaserScan and exit upon reception.
 */
void Jockey::getData()
{
  laserHandler_ = nh_.subscribe<sensor_msgs::LaserScan>("base_scan", 1, &Jockey::handleLaser, this);

  ros::Rate r(100);
  while (ros::ok())
  {
    if (data_received_)
    {
      // Stop the subscribers (may be superfluous).
      laserHandler_.shutdown();
      data_received_ = false;
      break;
    }
    ros::spinOnce();
    r.sleep();
  }
}

/* Receive a LaserScan message and store it.
 */
void Jockey::handleLaser(const sensor_msgs::LaserScanConstPtr& msg)
{
  scan_ = *msg;
  scan_reception_time_ = msg->header.stamp;
  data_received_ = true;
}

/* Return the vertex descriptors associated with the current robot position through result_.
 *
 * The descriptor are a LaserScan and a Crossing.
 */
// TODO: Discuss with Karel the exact role of onGetVertexDescriptor
// TODO: in particular: should it save something in the database? This jockey
// is not a learning jockey.
void Jockey::onGetVertexDescriptor()
{
  ROS_INFO("%s: Received action GET_VERTEX_DESCRIPTOR", ros::this_node::getName().c_str());

  if (server_.isPreemptRequested() && !ros::ok())
  {
    ROS_INFO("%s: Preempted", jockey_name_.c_str());
    // set the action state to preempted
    server_.setPreempted();
    return;
  }

  getData();

  // Add the LaserScan to the descriptor list.
  lama_interfaces::SetVectorLaserScan vscan_setter;
  vscan_setter.request.descriptor.push_back(scan_);
  if (!laser_descriptor_setter_.call(vscan_setter))
  {
    ROS_ERROR("Failed to add LaserScan[] to the map");
    server_.setAborted();
    return;
  }
  ROS_INFO("Added LaserScan[] with id %d", vscan_setter.response.id); // DEBUG
  result_.descriptor_links.push_back(laserDescriptorLink(vscan_setter.response.id));

  // Add the Crossing to the descriptor list.
  lama_msgs::SetCrossing crossing_setter;
  lama_msgs::Crossing crossing = crossing_detector_.crossingDescriptor(scan_, true);
  crossing_setter.request.descriptor = crossing;
  if (!crossing_descriptor_setter_.call(crossing_setter))
  {
    ROS_ERROR("Failed to add Crossing to the map");
    server_.setAborted();
    return;
  }
  ROS_INFO("Added Crossing with id %d", crossing_setter.response.id); // DEBUG
  result_.descriptor_links.push_back(crossingDescriptorLink(crossing_setter.response.id));
  
  result_.state = lama_jockeys::LocalizeResult::DONE;
  result_.completion_time = getCompletionDuration();
  server_.setSucceeded(result_);
}

void Jockey::onGetEdgesDescriptors()
{
  result_.state = lama_jockeys::LocalizeResult::NOT_SUPPORTED;
  result_.completion_time = ros::Duration(0.0);
  server_.setSucceeded(result_);
}

void Jockey::onLocalizeInVertex()
{
  result_.state = lama_jockeys::LocalizeResult::NOT_SUPPORTED;
  result_.completion_time = ros::Duration(0.0);
  server_.setSucceeded(result_);
}

void Jockey::onLocalizeEdge()
{
  result_.state = lama_jockeys::LocalizeResult::NOT_SUPPORTED;
  result_.completion_time = ros::Duration(0.0);
  server_.setSucceeded(result_);
}

void Jockey::onGetDissimilarity()
{
  ROS_INFO("%s: Received action GET_DISSIMILARITY", ros::this_node::getName().c_str());

  getData();

  // Transform the scan into a polygon.
  geometry_msgs::Polygon current_polygon = scanToPolygon(scan_);

  // Get all scans from database.
  lama_interfaces::ActOnMap srv;
  srv.request.action = lama_interfaces::ActOnMapRequest::GET_VERTEX_LIST;
  ROS_INFO("%s: calling action GET_VERTEX_LIST", ros::this_node::getName().c_str());
  if (map_agent_.call(srv))
  {
    ROS_INFO("%s: received response GET_VERTEX_LIST", ros::this_node::getName().c_str());
  }
  else
  {
    ROS_ERROR("%s: failed to call map agent", ros::this_node::getName().c_str());
    server_.setAborted();
    return;
  }
  
  // Iterate over vertices and get the associated Polygon (from the LaserScan).
  std::vector<int32_t> vertices;
  vertices.reserve(srv.response.objects.size());
  std::vector<geometry_msgs::Polygon> polygons;
  polygons.reserve(srv.response.objects.size());
  for (size_t i = 0; i < srv.response.objects.size(); ++i)
  {
    // Get all LaserScan descriptors associated with the current vertex.
    lama_interfaces::ActOnMap desc_srv;
    desc_srv.request.action = lama_interfaces::ActOnMapRequest::GET_DESCRIPTOR_LINKS;
    desc_srv.request.object.id = srv.response.objects[i].id;
    desc_srv.request.interface_name = laser_interface_name_;
    map_agent_.call(desc_srv);
    if (desc_srv.response.descriptor_links.empty())
    {
      continue;
    }
    if (desc_srv.response.descriptor_links.empty())
    {
      ROS_WARN("%s: more than one descriptor with interface %s for vertex %d, taking the first one",
          ros::this_node::getName().c_str(), laser_interface_name_.c_str(), desc_srv.request.object.id);
    }
    // Transform the LaserScan into a Polygon.
    lama_interfaces::GetVectorLaserScan scan_srv;
    scan_srv.request.id = desc_srv.response.descriptor_links[0].descriptor_id;
    if (!laser_descriptor_getter_.call(scan_srv))
    {
      ROS_ERROR("%s: failed to call %s service", ros::this_node::getName().c_str(),
          laser_interface_name_.c_str());
      server_.setAborted();
      return;
    }
    geometry_msgs::Polygon polygon = scanToPolygon(scan_srv.response.descriptor[0]);
    vertices.push_back(desc_srv.request.object.id);
    polygons.push_back(polygon);
  }
  
  // Compare them to the current polygon by calling one of the pm_* service.
  polygon_matcher::PolygonDissimilarity dissimi_srv;
  dissimi_srv.request.polygon1 = current_polygon;
  result_.idata.clear();
  result_.fdata.clear();
  result_.idata.reserve(vertices.size());
  result_.fdata.reserve(vertices.size());
  for (size_t i = 0; i < vertices.size(); ++i)
  {
    dissimi_srv.request.polygon2 = polygons[i];
    if (!dissimilarity_server_.call(dissimi_srv))
    {
      ROS_ERROR("%s: failed to call %s", ros::this_node::getName().c_str(),
          dissimilarity_server_name_.c_str());
      server_.setAborted();
      return;
    }
    result_.idata.push_back(vertices[i]);
    result_.fdata.push_back(dissimi_srv.response.raw_dissimilarity);
  }

  ROS_INFO("%s: computed %zu dissimilarities", jockey_name_.c_str(),
      result_.idata.size());
  result_.state = lama_jockeys::LocalizeResult::DONE;
  result_.completion_time = getCompletionDuration();
  server_.setSucceeded(result_);
}

lama_interfaces::DescriptorLink Jockey::laserDescriptorLink(const int32_t id)
{
  lama_interfaces::DescriptorLink descriptor_link;
  descriptor_link.descriptor_id = id;
  descriptor_link.interface_name = laser_interface_name_;
  return descriptor_link;
}

lama_interfaces::DescriptorLink Jockey::crossingDescriptorLink(const int32_t id)
{
  lama_interfaces::DescriptorLink descriptor_link;
  descriptor_link.descriptor_id = id;
  descriptor_link.interface_name = crossing_interface_name_;
  return descriptor_link;
}

} // namespace lj_laser
} // namespace lama
