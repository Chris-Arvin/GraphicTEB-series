/**
* Copyright 2014 Social Robotics Lab, University of Freiburg
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*    # Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*    # Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*    # Neither the name of the University of Freiburg nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* \author Billy Okal <okal@cs.uni-freiburg.de>
* \author Sven Wehner <mail@svenwehner.de>
*/

#include <pedsim_simulator/element/agent.h>
#include <pedsim_simulator/element/agentcluster.h>
#include <pedsim_simulator/element/areawaypoint.h>
#include <pedsim_simulator/element/attractionarea.h>
#include <pedsim_simulator/element/obstacle.h>
#include <pedsim_simulator/element/waitingqueue.h>
#include <pedsim_simulator/scenarioreader.h>

#include <QFile>
#include <iostream>

#include <ros/ros.h>

ScenarioReader::ScenarioReader() {
  // initialize values
  currentAgents = nullptr;
  currentSpawnArea = nullptr;
}

bool ScenarioReader::readFromFile(const QString& filename) {
  ROS_DEBUG("Loading scenario file '%s'.", filename.toStdString().c_str());

  // open file
  QFile file(filename);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    ROS_DEBUG("Couldn't open scenario file!");
    return false;
  }

  // read input
  xmlReader.setDevice(&file);

  while (!xmlReader.atEnd()) {
    xmlReader.readNext();
    processData();
  }

  // check for errors
  if (xmlReader.hasError()) {
    // TODO - fix qstring and std string issues here to show error lines
    ROS_DEBUG("Error while reading scenario file");
    return false;
  }

  // report success
  return true;
}

void ScenarioReader::processData() {
  if (xmlReader.isStartElement()) {
    const QString elementName = xmlReader.name().toString();
    const QXmlStreamAttributes elementAttributes = xmlReader.attributes();

    if ((elementName == "scenario") || (elementName == "welcome")) {
      // nothing to do
    } else if (elementName == "obstacle") {
      if (elementAttributes.value("type").toString() == "line"){
        const double x1 = elementAttributes.value("x1").toString().toDouble();
        const double y1 = elementAttributes.value("y1").toString().toDouble();
        const double x2 = elementAttributes.value("x2").toString().toDouble();
        const double y2 = elementAttributes.value("y2").toString().toDouble();
        for (const auto& cell : LineObstacleToCells(x1, y1, x2, y2)) {
          Obstacle* obs = new Obstacle(cell[0], cell[1], cell[2], cell[3]);
          SCENE.addObstacle(obs);
        }
      }
      else if (elementAttributes.value("type").toString() == "rectangle"){
        const double x = elementAttributes.value("x").toString().toDouble();
        const double y = elementAttributes.value("y").toString().toDouble();
        const double xHalfLength = elementAttributes.value("xHalfLength").toString().toDouble();
        const double yHalfLength = elementAttributes.value("yHalfLength").toString().toDouble();
        for (const auto& cell : RectangleObstacleToCells(x, y, xHalfLength, yHalfLength)) {
          Obstacle* obs = new Obstacle(cell[0], cell[1], cell[2], cell[3]);
          SCENE.addObstacle(obs);
        }
      }

      else if (elementAttributes.value("type").toString() == "circle"){
        const double x = elementAttributes.value("x").toString().toDouble();
        const double y = elementAttributes.value("y").toString().toDouble();
        const double radius = elementAttributes.value("radius").toString().toDouble();
        for (const auto& cell : CircleObstacleToCells(x, y, radius)) {
          Obstacle* obs = new Obstacle(cell[0], cell[1], cell[2], cell[3]);
          SCENE.addObstacle(obs);
        }
      }

    } else if (elementName == "waypoint") {
      const QString id = elementAttributes.value("id").toString();
      const double x = elementAttributes.value("x").toString().toDouble();
      const double y = elementAttributes.value("y").toString().toDouble();
      const double r = elementAttributes.value("r").toString().toDouble();
      // TODO - make the setting of waypoint behavior optional, 
      // and default to SIMPLE.
      const int b = elementAttributes.value("b").toString().toInt();
      AreaWaypoint* w = new AreaWaypoint(id, x, y, r);
      w->setBehavior(static_cast<Ped::Twaypoint::Behavior>(b));
      SCENE.addWaypoint(w);
    } else if (elementName == "queue") {
      const QString id = elementAttributes.value("id").toString();
      const double x = elementAttributes.value("x").toString().toDouble();
      const double y = elementAttributes.value("y").toString().toDouble();
      const double directionValue =
          elementAttributes.value("direction").toString().toDouble();

      const Ped::Tvector position(x, y);
      const Ped::Tangle direction = Ped::Tangle::fromDegree(directionValue);

      WaitingQueue* queue = new WaitingQueue(id, position, direction);
      SCENE.addWaitingQueue(queue);
    } else if (elementName == "attraction") {
      const QString id = elementAttributes.value("id").toString();
      const double x = elementAttributes.value("x").toString().toDouble();
      const double y = elementAttributes.value("y").toString().toDouble();
      const double width =
          elementAttributes.value("width").toString().toDouble();
      const double height =
          elementAttributes.value("height").toString().toDouble();
      const double strength =
          elementAttributes.value("strength").toString().toDouble();

      AttractionArea* attraction = new AttractionArea(id);
      attraction->setPosition(x, y);
      attraction->setSize(width, height);
      attraction->setStrength(strength);
      SCENE.addAttraction(attraction);
    } else if (elementName == "agent") {
      const double x = elementAttributes.value("x").toString().toDouble();
      const double y = elementAttributes.value("y").toString().toDouble();
      const int n = elementAttributes.value("n").toString().toInt();
      const double dx = elementAttributes.value("dx").toString().toDouble();
      const double dy = elementAttributes.value("dy").toString().toDouble();
      const int type = elementAttributes.value("type").toString().toInt();
      AgentCluster* agentCluster = new AgentCluster(x, y, n);
      agentCluster->setDistribution(dx, dy);

      /// TODO - change agents Vmax distribution based on agent type
      /// and other force parameters to realize different behaviours
      agentCluster->setType(static_cast<Ped::Tagent::AgentType>(type));
      SCENE.addAgentCluster(agentCluster);
      currentAgents = agentCluster;
    } else if (elementName == "source") {
      const double x = elementAttributes.value("x").toString().toDouble();
      const double y = elementAttributes.value("y").toString().toDouble();
      const int n = elementAttributes.value("n").toString().toInt();
      const double dx = elementAttributes.value("dx").toString().toDouble();
      const double dy = elementAttributes.value("dy").toString().toDouble();
      const int type = elementAttributes.value("type").toString().toInt();
      AgentCluster* agentCluster = new AgentCluster(x, y, n);
      agentCluster->setDistribution(dx, dy);
      agentCluster->setType(static_cast<Ped::Tagent::AgentType>(type));
      SCENE.addAgentCluster(agentCluster);
      currentAgents = agentCluster;

      SpawnArea* spawn_area = new SpawnArea(x, y, n, dx, dy);
      SCENE.addSpawnArea(spawn_area);
      currentSpawnArea = spawn_area;
    } else if (elementName == "addwaypoint") {
      if (currentAgents == nullptr) {
        ROS_DEBUG("Invalid <addwaypoint> element outside of agent element!");
        return;
      }

      // add waypoints to current <agent> element
      QString id = elementAttributes.value("id").toString();
      currentAgents->addWaypoint(SCENE.getWaypointByName(id));

      if (currentSpawnArea != nullptr) {
        currentSpawnArea->waypoints.emplace_back(id);
      }
    } else if (elementName == "addqueue") {
      if (currentAgents == nullptr) {
        ROS_DEBUG("Invalid <addqueue> element outside of agent element!");
        return;
      }

      // add waiting queue to current <agent> element
      QString id = elementAttributes.value("id").toString();
      currentAgents->addWaitingQueue(SCENE.getWaitingQueueByName(id));
    } else {
      // inform the user about invalid elements
      ROS_DEBUG("Unknown element: <%s>", elementName.toStdString().c_str());
    }
  } else if (xmlReader.isEndElement()) {
    const QString elementName = xmlReader.name().toString();

    if (elementName == "agent") {
      currentAgents = nullptr;
    }
  }
}

std::vector<std::vector<float>> ScenarioReader::LineObstacleToCells(const float x1,
                                                         const float y1,
                                                         const float x2,
                                                         const float y2) {
  std::vector<std::vector<float>> obstacle_cells;
  float dx = x2 - x1;
  float dy = y2 - y1;
  float step = std::fabs(dx) >= std::fabs(dy) ? std::fabs(dx) : std::fabs(dy);
  step*=10;
  float x = x1;
  float y = y1;
  float xOld = x1;
  float yOld = y1;
  if (step > 0) {
    dx /= step;
    dy /= step;
    int i = 1;
    while (i <= step) {
      x = x + dx;
      y = y + dy;
      i = i + 1;
      vector<float> temp;
      temp.push_back(xOld);
      temp.push_back(yOld);
      temp.push_back(x);
      temp.push_back(y);
      obstacle_cells.push_back(temp);
      xOld = x;
      yOld = y;
    }
    if (x!=x2 || y!=y2){
      vector<float> temp;
      temp.push_back(x);
      temp.push_back(y);
      temp.push_back(x2);
      temp.push_back(y2);
      obstacle_cells.push_back(temp);
    }
  }
  return obstacle_cells;
}


std::vector<std::vector<float>> ScenarioReader::RectangleObstacleToCells(const float x,
                                                         const float y,
                                                         const float xHalfLength,
                                                         const float yHalfLength) {
  std::vector<std::vector<float>> obstacle_cells;
  float dx = xHalfLength*2;
  float dy = yHalfLength*2;
  float xStep = std::fabs(dx)*10;
  float yStep = std::fabs(dy)*10;
  float xStart = x-xHalfLength;
  float yStart = y-yHalfLength;
  float xTarget = x+xHalfLength;
  float yTarget = y+yHalfLength;
  float xOld = xStart;
  float yOld = yStart;
  if (xStep > 0 || yStep >0) {
    dx /= xStep;
    dy /= yStep;
    int xIndex = 1;
    while (xIndex <= xStep+1) {
      int yIndex = 1;
      xIndex ++;
      yStart = y-yHalfLength;
      yOld = yStart;
      while (yIndex<=yStep+1){
        yIndex ++;
        vector<float> temp;
        temp.push_back(xOld);
        temp.push_back(yOld);
        temp.push_back(xStart);
        temp.push_back(yStart);
        obstacle_cells.push_back(temp);
        xOld = xStart;
        yOld = yStart;
        yStart += dy;
      }
      
      if (yStart!=yTarget){
        vector<float> temp;
        temp.push_back(xOld);
        temp.push_back(yOld);
        temp.push_back(xStart);
        temp.push_back(yTarget);
        obstacle_cells.push_back(temp);
      }
      xStart += dx;
    }
    
    
    if (xStart!=xTarget){
      int yIndex = 1;
      xStart = xTarget;
      xOld = xStart;
      xIndex ++;
      yStart = y-yHalfLength;
      yOld = yStart;
      while (yIndex<=yStep+1){
        yIndex ++;
        vector<float> temp;
        temp.push_back(xOld);
        temp.push_back(yOld);
        temp.push_back(xStart);
        temp.push_back(yStart);
        obstacle_cells.push_back(temp);
        xOld = xStart;
        yOld = yStart;
        yStart += dy;
      }
      if (yStart!=yTarget){
        vector<float> temp;
        temp.push_back(xOld);
        temp.push_back(yOld);
        temp.push_back(xStart);
        temp.push_back(yTarget);
        obstacle_cells.push_back(temp);
      }
    }
  }
  return obstacle_cells;
}


std::vector<std::vector<float>> ScenarioReader::CircleObstacleToCells(const float x,
                                                         const float y,
                                                         const float Radius) {
  std::vector<std::vector<float>> obstacle_cells;
  float dx = Radius*2;
  float dy = Radius*2;
  float xStep = std::fabs(dx)*10;
  float yStep = std::fabs(dy)*10;
  float xStart = x-Radius;
  float yStart = y-Radius;
  float xEnd = x+Radius;
  float yEnd = y+Radius;

  if (xStep > 0 || yStep >0) {
    dx /= xStep;
    dy /= yStep;
    float xIter = xStart;
    while (xIter <= xEnd){
      float yIter = yStart;
      float y1=-100;
      float y2=-100;
      while (yIter <= yEnd){
        if (std::hypot(xIter-x, yIter-y) <= Radius+0.01){
          if (y1==-100)
            y1 = yIter;
          y2 = yIter;
        }
        yIter += dy;
      }

      if (y1!=-100 && y2!=-100){
        for (float ytemp=y1;ytemp<=y2;ytemp+=dy){
          vector<float> temp;
          temp.push_back(xIter);
          temp.push_back(ytemp);
          temp.push_back(xIter);
          temp.push_back(ytemp);     
          obstacle_cells.push_back(temp);
        }
      }
      xIter += dx;
    }
  }
  return obstacle_cells;
}


