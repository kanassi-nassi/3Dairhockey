#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/rendering/DynamicLines.hh>
#include <ros/ros.h>
#include <gazebo_msgs/GetModelState.h>

namespace gazebo
{
  class CablePlugin : public ModelPlugin
  {
  public:
    CablePlugin() : ModelPlugin() {}

    void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
    {
      this->model = _model;
      this->world = this->model->GetWorld();
      this->scene = rendering::get_scene();
      if (!this->scene)
      {
        gzerr << "Unable to get scene" << std::endl;
        return;
      }

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&CablePlugin::OnUpdate, this));

      this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
      this->getStateClient = this->rosNode->serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

      // 固定点のモデル名
      this->fixedPoints = {
          "fixed_point1", "fixed_point2", "fixed_point3", "fixed_point4",
          "fixed_point5", "fixed_point6", "fixed_point7", "fixed_point8"};

      // 固定点のVisualを取得
      for (const auto &point_name : this->fixedPoints)
      {
        rendering::VisualPtr fixedPointVisual = this->scene->GetVisual(point_name);
        if (fixedPointVisual)
        {
          this->fixedPointVisuals.push_back(fixedPointVisual);
        }
        else
        {
          gzerr << "Unable to get visual for " << point_name << std::endl;
        }
      }

      // エンドエフェクタのVisualを取得
      this->endEffectorVisual = this->scene->GetVisual("bouncing_object");
      if (!this->endEffectorVisual)
      {
        gzerr << "Unable to get visual for bouncing_object" << std::endl;
      }
    }

    void OnUpdate()
    {
      if (!this->endEffectorVisual)
      {
        return;
      }

      ignition::math::Vector3d endEffectorPos = this->endEffectorVisual->WorldPose().Pos();

      for (size_t i = 0; i < this->fixedPointVisuals.size(); ++i)
      {
        rendering::VisualPtr fixedPointVisual = this->fixedPointVisuals[i];
        ignition::math::Vector3d fixedPointPos = fixedPointVisual->WorldPose().Pos();

        // Create a visual to attach the line to
        std::string lineName = "cable_line_" + std::to_string(i);
        rendering::VisualPtr lineVisual(new rendering::Visual(lineName, this->scene->WorldVisual()));
        this->scene->AddVisual(lineVisual);

        rendering::DynamicLines *line = lineVisual->CreateDynamicLine(rendering::RENDERING_LINE_LIST);
        line->AddPoint(fixedPointPos);
        line->AddPoint(endEffectorPos);
        line->setMaterial("Gazebo/Red");
        line->setVisibilityFlags(GZ_VISIBILITY_GUI);
        lineVisual->SetVisible(true);
      }
    }

  private:
    physics::ModelPtr model;
    physics::WorldPtr world;
    rendering::ScenePtr scene;
    event::ConnectionPtr updateConnection;
    std::unique_ptr<ros::NodeHandle> rosNode;
    ros::ServiceClient getStateClient;
    std::vector<std::string> fixedPoints;
    std::vector<rendering::VisualPtr> fixedPointVisuals;
    rendering::VisualPtr endEffectorVisual;
  };

  GZ_REGISTER_MODEL_PLUGIN(CablePlugin)
}