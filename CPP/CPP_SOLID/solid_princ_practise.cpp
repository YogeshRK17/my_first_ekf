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

//O

class Invoice
{
public:
    std::string type;

    Invoice(std::string t) : type(t) {}

    double get_discount(double amount)
    {
        if (type == "regular") return amount*0.05;
        if (type == "premium") return amount*0.2;
        else return 0.0;
    }

};

//voilation of OCP, because in future if another different type of users gets added it may create a need to modify class

//let's use OOPs and solve this problem


//first let's define abstract class for discount policy
class discountPolicy
{
    virtual double getDiscount(double amount) const = 0;
    virtual ~discountPolicy() {}                          //good to have virtual destructor to avoid double copies
};

class regularDiscount : public discountPolicy
{
public:
    double getDiscount(double amount) const override
    {
        return amount*0.05;
    }
};

class premiumDiscount : public discountPolicy
{
public:
    double getDiscount(double amount) const override
    {
        return amount*0.2;
    }
};

//likewise we can add more and more users

class Invoice_OCP
{
    const discountPolicy& DiscountPolicy;
public:
    Invoice_OCP(const discountPolicy& dp): DiscountPolicy(dp) {}

    void printTotal(double amount) const{
        std::cout << "Discount: " << DiscountPolicy.getDiscount(amount) <<std::endl;
    }
};

int main()
{
    premiumDiscount premDiscount;
    Invoice_OCP inv(premDiscount);
    inv.printTotal(1000.0);
}


//L

//Liskov substitution principle says a subclass should get substituted parent class

//Pass by reference 
// // You pass a address(variable/object) to method/function with '&' sign. In this case method/function will get original value not copy.
// // Means here you can modify original value not copy.


//Pass by pointer
// // You pass a pointer(variable/object) to method/function. you can access object methods using '->'.

//I

//Avoid using Fat interfaces/abstract class, Instead use small interfaces. i.e. segregate interfaces into small chunks

class Iprint
{
public:
    virtual void print() = 0;
    virtual void scan()  = 0;
    virtual void fax()   = 0;
    virtual ~Iprint(){};
};

class OldPrinter : public Iprint
{
    void print() override{
        std::cout<<"Printing..."<<std::endl;
    }

    void scan() override{
        throw std::runtime_error("scan not supported");
    }

    void fax() override{
        throw std::runtime_error("Fax not supported");
    }

};

//Voilation of ISP as many interfaces from parent class haven't been used

//Below is application of ISP, where in each interface is segregated

class Iprinting
{
public:
    virtual void print() = 0;
};

class Iscanning
{
public:
    virtual void scan() = 0;
};

class Ifaxing
{
public:
    virtual void fax() = 0;
};

class oldPrinter: public Iprinting
{
public:
    void print() override{
        std::cout<<"Printing..."<<std::endl;
    }

};

class newPrinter: public Iprinting, Iscanning, Ifaxing
{
public:
    void print() override{
        std::cout<<"Printing..."<<std::endl;
    }

    void scan() override{
        std::cout<<"Scanning..."<<std::endl;
    }

    void fax() override{
        std::cout<<"Faxing..."<<std::endl;
    }

};

//D
//Dependency Inversion Principle, High level modules should not depends on low level modules.
//Use abstract classes instead of single concrete class.

class db
{
public:
    virtual void initialize()  = 0;
    virtual void access_data() = 0;
    virtual void put_data()    = 0;
    virtual void erase_data()  = 0;
    virtual ~db() {}
};

class mongodb : public db
{
public:
    void initialize() override{
        std::cout<<"Initializing mongodb database"<<std::endl;
    }

    //Likewise we can add other methods from parent db class

};



