//
// Created by Stanislav Olekhnovich on 02/08/2017.
//

#include "WebSocketMessageHandler.h"

string WebSocketMessageHandler::CreateResponseMessage(const std::vector<CartesianPoint>& path)
{
    json msgJson;

    std::vector<double> pathX;
    std::vector<double> pathY;
    for (auto& p: path)
    {
        pathX.push_back(p.X);
        pathY.push_back(p.Y);
    }

    msgJson["next_x"] = pathX;
    msgJson["next_y"] = pathY;

    auto msg = "42[\"control\"," + msgJson.dump() + "]";
    return msg;
}

string WebSocketMessageHandler::ProcessMessageContent(string& content)
{
    auto jsonContent = json::parse(content);
    string eventType = jsonContent[0].get<string>();

    string response;
    if (eventType == "telemetry")
    {
        auto data = jsonContent[1];
        auto pathPlannerInput = ReadPlannerInput(data);
        auto output = pathPlanner.GeneratePath(pathPlannerInput);
        response = CreateResponseMessage(output);
    }
    return response;
}

PathPlannerInput WebSocketMessageHandler::ReadPlannerInput(json data)
{
    PathPlannerInput pathPlannerInput;

    pathPlannerInput.LocationCartesian= { data["x"], data["y"], data["yaw"] };
    pathPlannerInput.LocationFrenet= { data["s"], data["d"] };
    pathPlannerInput.Speed = data["speed"];
    pathPlannerInput.PreviousPathX = data["previous_path_x"].get<std::vector<double>>();
    pathPlannerInput.PreviousPathY = data["previous_path_y"].get<std::vector<double>>();

    assert(pathPlannerInput.PreviousPathX.size() == pathPlannerInput.PreviousPathY.size());
    for (int i = 0; i < pathPlannerInput.PreviousPathX.size(); i++)
    {
        pathPlannerInput.Path.emplace_back(pathPlannerInput.PreviousPathX[i], pathPlannerInput.PreviousPathY[i]);
    }

    pathPlannerInput.PathEndpointFrenet = { data["end_path_s"], data["end_path_d"] };
    auto sensorFusionData = data["sensor_fusion"].get<std::vector<std::vector<double>>>();
    for (auto& otherCarData : sensorFusionData)
    {
        OtherCar otherCar;
        otherCar.LocationCartesian = { otherCarData[1], otherCarData[2] };
        otherCar.XAxisSpeed = otherCarData[3];
        otherCar.YAxisSpeed = otherCarData[4];
        otherCar.LocationFrenet = { otherCarData[5], otherCarData[6] };

        pathPlannerInput.OtherCars.push_back(otherCar);
    }

    return pathPlannerInput;
}

string WebSocketMessageHandler::GetMessageContent(const string& message)
{
    string content;

    bool hasNullContent = (message.find("null") != string::npos);
    if (hasNullContent)
        return content;

    auto b1 = message.find_first_of('[');
    auto b2 = message.find_last_of(']');

    if (b1 != string::npos && b2 != string::npos)
        content = message.substr(b1, b2 - b1 + 1);

    return content;
}

bool WebSocketMessageHandler::MessageHasExpectedPrefix(const string& message)
{
    // "42" at the start of the message means there's a websocket message event.
    const string prefix {"42"};
    return (message.substr(0, prefix.size()) == prefix);
}

void WebSocketMessageHandler::HandleMessage(const string& message, uWS::WebSocket<uWS::SERVER>& ws)
{
    if (!MessageHasExpectedPrefix(message))
        return;

    auto content = GetMessageContent(message);
    if (content.empty())
    {
        SendDefaultResponse(ws);
        return;
    }

    auto response = ProcessMessageContent(content);

    if (!response.empty())
        ws.send(response.data(), response.length(), uWS::OpCode::TEXT);
}

void WebSocketMessageHandler::SendDefaultResponse(uWS::WebSocket<uWS::SERVER>& ws) const
{
    string response = "42[\"manual\",{}]";
    std::cout << response << std::endl;
    ws.send(response.data(), response.length(), uWS::TEXT);
}


