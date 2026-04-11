#include "FileHandler.h"
#include <iostream>
#include <stdexcept>

FileHandler::FileHandler(const std::string& filename,const std::string& mode) {
    file_ptr = std::fopen(filename.c_str(),mode.c_str());
    if (!file_ptr) {
        throw std::runtime_error("Failed to open file: " + filename);
    }
    std::cout << "File opened successfully: " << filename << std::endl;
}

FileHandler::~FileHandler() {
    if (file_ptr) {
        std::fclose(file_ptr);
        std::cout << "File closed successfully." << std::endl;
    }
}

FileHandler::FileHandler(FileHandler&& other) noexcept : file_ptr(other.file_ptr) {
    other.file_ptr = nullptr;
    std::cout << "FileHandler moved (constructor)." << std::endl;
}

FileHandler& FileHandler::operator=(FileHandler&& other) noexcept {
    if (this !=&other) {
        if (file_ptr) {
            std::fclose(file_ptr);
        }
        file_ptr = other.file_ptr;
        other.file_ptr = nullptr;
        std::cout << "FileHandler moved (assignment)." << std::endl;
    }
    return *this;
}
std::FILE* FileHandler::get() const {
    return file_ptr;
}
bool FileHandler::isValid() const {
    return file_ptr != nullptr;
}