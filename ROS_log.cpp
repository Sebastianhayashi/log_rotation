#include <iostream>
#include <string>
#include <filesystem>
#include <rclcpp>

int main(){
    std::string command = "ros2 node list";
    FILE* process = popen(command, "r")

    char buffer[1024];

    while (fgest(buffer,sizeof(buffer), process) != NULL){
        node_name.push_back(buffer);
    }
    pclose(process);
}