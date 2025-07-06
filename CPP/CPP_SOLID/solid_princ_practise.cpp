//let's practise cpp, Here we are going to see bad code and good code examples, using SOLID principles

#include <iostream>
#include <fstream>
#include <string>

//S

class Report
{
public:
    std::string title;
    std::string content;

    Report(std::string t, std::string c) : title(t), content(t) {}

    void print()
    {
        std::cout<<title<<'\n'<<content<<std::endl;
    }

    void saveToFile(const std::string & filename)
    {
        std::ofstream file(filename);
        file << title << '\n' << content;
        file.close();
    }

private:
};

//voilation of S principle, because class Report has many responsibilities like content_reading, printing, file_saving etc

//let's apply S principle and segrigate their responsibilities

class readReport
{
public:
    std::string title;
    std::string content;

    readReport(std::string t, std::string c): title(t), content(c) {}
    
};

class printReport : public readReport
{
public:
    void print()
    {
        std::cout << title << '\n' << content << std::endl;
    }

};

class saveReport : public readReport
{
public:
    void save_report(const std::string & filename)
    {
        std::ofstream file(filename);
        file << title << '\n' << content;
        file.close();
    }

};
