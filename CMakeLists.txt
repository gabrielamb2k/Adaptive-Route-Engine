cmake_minimum_required(VERSION 3.15)

if(WIN32)
    set(VCPKG_TARGET_TRIPLET "x64-mingw-dynamic")
endif()

project(ProjetoRotas)

find_package(Crow CONFIG REQUIRED)

add_executable(meu_servidor servTest.cpp)

# --- CORREÇÃO AQUI ---
# Adicionamos 'ws2_32' e 'wsock32' no final
target_link_libraries(meu_servidor PRIVATE Crow::Crow ws2_32 wsock32)