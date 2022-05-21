#include<algorithm>
#include<array>
#include<stdlib.h>
#include<string>
#include<bitset>

#include"robot.h"
#include"plan.h"
#include"kinematics.h"

double input_angle[18] = { 0 };
double init_pos_angle[18] = { 0 };

//输出参数，模型曲线测试使用
double file_current_leg[18] = { 0 };
double file_current_body[16] = { 0 };
double time_test = 0;
extern double PI;

using namespace aris::dynamic;
using namespace aris::plan;
namespace robot
{

//---------------------读取当前电机的位置--------------------//
auto ReadCurrentPos::prepareNrt()->void
{
    for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto ReadCurrentPos::executeRT()->int
{
    double current_pos[18] = { 0 };
    for (int i = 0; i < 3; ++i) {
        this->master()->logFileRawName("CurrentPos");
        current_pos[i] = controller()->motionPool()[i].actualPos();
        mout() << current_pos[i] << std::endl;
        lout() << current_pos[i] << std::endl;
    }

    return 0;
}
auto ReadCurrentPos::collectNrt()->void {}
ReadCurrentPos::ReadCurrentPos(const std::string& name)
{
    aris::core::fromXmlString(command(),
        "<Command name=\"read\">"
        "</Command>");
}
ReadCurrentPos::~ReadCurrentPos() = default;

//---------------------TCurve2 test--------------------//
auto TCurve2Test::prepareNrt()->void
{
    for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto TCurve2Test::executeRT()->int
{
    if (count() == 1) {
        this->master()->logFileRawName("test");
    }
    TCurve2 s1(1, 2, 10);
    s1.getCurveParam();
    int ret = s1.getTc() * 1000 - count();
    //std::cout << "Tc = " << s1.getTc() << std::endl;
    //std::cout << "ta = " << s1.getta() << std::endl;
    //std::cout << "v = " << s1.getv() << std::endl;
    //std::cout << "a = " << s1.geta() << std::endl;
    std::cout << "count = " << count() << std::endl;
    std::cout << "ret = " << ret << std::endl;
    std::cout << s1.getTCurve(count()) << std::endl;
    std::cout << std::endl;
    lout() << s1.getTCurve(count()) << std::endl;
    return ret;
}


auto TCurve2Test::collectNrt()->void {}
TCurve2Test::TCurve2Test(const std::string& name)
{
    aris::core::fromXmlString(command(),
        "<Command name=\"test\">"
        "</Command>");
}
TCurve2Test::~TCurve2Test() = default;




//---------------------home指令--------------------//
//在极限位置上电
auto Home::prepareNrt()->void
{
    for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto Home::executeRT()->int
{

    TCurve s1(2, 2);
    s1.getCurveParam();
    int time = s1.getTc() * 1000;

    static double begin_angle[18];
    if (count() == 1) {
        for (int i = 0; i < 3; ++i) {
            begin_angle[i] = controller()->motionPool()[i].targetPos(); //这里的位置应该是0
        }
    }


    double current_angle[18] = { 0 };
    for (int i = 0; i < 3; ++i) {
        current_angle[i] = begin_angle[i] - (begin_angle[i] - pos_offset[i]) * s1.getTCurve(count()) ; //电机的绝对值为pos_offset

       mout() << begin_angle[0] << "\t" << current_angle[0] << std::endl;



        controller()->motionPool()[i].setTargetPos(current_angle[i]);
      //  mout() << current_angle[i] << std::endl;
    }
//    if (count() % 10 == 0) {
//        for (int i = 0; i < 3; ++i) {
//            mout() << controller()->motionPool()[i].actualPos() << "\t";
//        }
//        mout() << std::endl;
//    }
    int ret = time - count();
    return ret;
}
auto Home::collectNrt()->void {}
Home::Home(const std::string& name)
{
    aris::core::fromXmlString(command(),
        "<Command name=\"home\">"
        "</Command>");
}
Home::~Home() = default;

//---------------------home2指令--------------------//
//在零位上电,home位置所有电机均为0
auto Home2::prepareNrt()->void
{
    for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto Home2::executeRT()->int
{

    TCurve s1(1, 1);
    s1.getCurveParam();
    int time = s1.getTc() * 1000;

    static double begin_angle[18];
    if (count() == 1) {
        for (int i = 0; i < 3; ++i) {
            begin_angle[i] = controller()->motionPool()[i].targetPos();
        }
    }


    double current_angle[18] = { 0 };
    for (int i = 0; i < 3; ++i) {
        current_angle[i] = begin_angle[i] - (begin_angle[i]) * s1.getTCurve(count()) ; //电机的绝对值为pos_offset
        controller()->motionPool()[i].setTargetPos(current_angle[i]);
        mout() << current_angle[i] << std::endl;
    }
    if (count() % 10 == 0) {
        for (int i = 0; i < 3; ++i) {
            mout() << controller()->motionPool()[i].actualPos() << "\t";
        }
        mout() << std::endl;
    }
    int ret = time - count();
    return ret;
}
auto Home2::collectNrt()->void {}
Home2::Home2(const std::string& name)
{
    aris::core::fromXmlString(command(),
        "<Command name=\"home2\">"
        "</Command>");
}
Home2::~Home2() = default;


//---------------------每个电机简单性能测试（梯形曲线移动）--------------------//
auto MoveJointAll::prepareNrt()->void
{
    cef_ = doubleParam("coefficient");
    for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto MoveJointAll::executeRT()->int
{
    static double begin_angle[18] = { 0 };
    if (count() == 1) {
        for (int i = 0; i < 3; ++i) {
            begin_angle[i] = controller()->motionPool()[i].targetPos();
        }
    }

    double d = cef_ * 10;
    TCurve2 s1(5, 2, d);
    s1.getCurveParam();
    //int time = s1.getTc() * 1000;

    double angle[18] = { 0 };
    for (int i = 0; i < 3; ++i) {
        angle[i] = begin_angle[i] +  s1.getTCurve(count());
        controller()->motionPool()[i].setTargetPos(angle[i]);
    }
    //int ret = time - count();
    //std::cout << "ret = " << ret << std::endl;
    return s1.getTc()*1000 -count();
}


auto MoveJointAll::collectNrt()->void {}
MoveJointAll::MoveJointAll(const std::string& name)
{
    aris::core::fromXmlString(command(),
        "<Command name=\"moveJA\">"
        "<Param name=\"coefficient\" default=\"1.0\" abbreviation=\"c\"/>"
        "</Command>");
}
MoveJointAll::~MoveJointAll() = default;


// ---------------------------单关节正弦往复轨迹 --------------------------------//
struct MoveJSParam1
{
    double j1;
    double time;
    uint32_t timenum;
};
auto Test::prepareNrt()->void
{
    MoveJSParam1 param;

    param.j1 = 0.0;
    param.time = 0.0;
    param.timenum = 0;

    for (auto &p : cmdParams())
    {
        if (p.first == "j1")
        {
            if (p.second == "current_pos")
            {
                param.j1 = controller()->motionPool()[0].actualPos();
            }
            else
            {
                param.j1 = doubleParam(p.first);
            }

        }
        else if (p.first == "time")
        {
            param.time = doubleParam(p.first);
        }
        else if (p.first == "timenum")
        {
            param.timenum = int32Param(p.first);
        }
    }
    this->param() = param;
    std::vector<std::pair<std::string, std::any>> ret_value;
    for (auto &option : motorOptions())	option |= NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER|NOT_CHECK_POS_CONTINUOUS;
    ret() = ret_value;
}
auto Test::executeRT()->int
{

    auto &param = std::any_cast<MoveJSParam1&>(this->param());
    auto time = static_cast<int32_t>(param.time * 1000);
    auto totaltime = static_cast<int32_t>(param.timenum * time);
    static double begin_pjs;
    static double step_pjs;
    // 访问主站 //
    auto &cout = controller()->mout();

    if ((1 <= count()) && (count() <= time / 2))
    {
        // 获取当前起始点位置 //
        if (count() == 1)
        {
            begin_pjs = controller()->motionPool()[0].actualPos();
            step_pjs = controller()->motionPool()[0].actualPos();
            this->master()->logFileRawName("moveJS");//建立记录数据的文件夹
        }
        step_pjs = begin_pjs + param.j1 * (1 - std::cos(2 * PI*count() / time)) / 2;
        controller()->motionPool().at(0).setTargetPos(step_pjs);
    }
    else if ((time / 2 < count()) && (count() <= totaltime - time / 2))
    {
        // 获取当前起始点位置 //
        if (count() == time / 2 + 1)
        {
            begin_pjs = controller()->motionPool()[0].actualPos();
            step_pjs = controller()->motionPool()[0].actualPos();
        }

        step_pjs = begin_pjs - 2 * param.j1 * (1 - std::cos(2 * PI*(count() - time / 2) / time)) / 2;
        controller()->motionPool().at(0).setTargetPos(step_pjs);
    }
    else if ((totaltime - time / 2 < count()) && (count() <= totaltime))
    {
        // 获取当前起始点位置 //
        if (count() == totaltime - time / 2 + 1)
        {
            begin_pjs = controller()->motionPool()[0].actualPos();
            step_pjs = controller()->motionPool()[0].actualPos();
        }
        step_pjs = begin_pjs - param.j1 * (1 - std::cos(2 * PI*(count() - totaltime + time / 2) / time)) / 2;
        controller()->motionPool().at(0).setTargetPos(step_pjs);
    }

    // 打印 //
    if (count() % 10 == 0)
    {
        mout() << "pos" << ":" << controller()->motionAtAbs(0).actualPos() << "\t";
        mout() << "vel" << ":" << controller()->motionAtAbs(0).actualVel() << std::endl;
    }

    // log //
//    auto &lout = controller()->lout();
//    lout << controller()->motionAtAbs(0).targetPos() << ",";
//    lout << std::endl;
    lout() << controller()->motionAtAbs(0).actualPos() <<"\t";
    lout() << controller()->motionAtAbs(0).actualVel() <<std::endl;

    return totaltime - count();
}
auto Test::collectNrt()->void {}
Test::Test(const std::string &name)
{
    aris::core::fromXmlString(command(),
        "<Command name=\"test1\">"
        "	<GroupParam>"
        "		<Param name=\"j1\" default=\"current_pos\"/>"
        "		<Param name=\"time\" default=\"4.0\" abbreviation=\"t\"/>"
        "		<Param name=\"timenum\" default=\"2\" abbreviation=\"n\"/>"
        "	</GroupParam>"
        "</Command>");
}
Test::~Test() = default;

//---------------------每个电机简单性能测试（cos曲线移动）--------------------//
struct MoveJSParam
{
    double amplitude;
    double time;
    uint32_t timenum;
};
auto MoveJointAllCos::prepareNrt()->void
{
    MoveJSParam param;

    param.amplitude = 0.0;
    param.time = 0.0;
    param.timenum = 0;

    for (auto& p : cmdParams())
    {
        if (p.first == "amplitude")
        {
            param.amplitude = doubleParam(p.first);
        }
        else if (p.first == "time")
        {
            param.time = doubleParam(p.first);
        }
        else if (p.first == "timenum")
        {
            param.timenum = int32Param(p.first);
        }
    }
    this->param() = param;
    std::vector<std::pair<std::string, std::any>> ret_value;
    for (auto& option : motorOptions())	option |= NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER | NOT_CHECK_POS_CONTINUOUS;
    ret() = ret_value;
}
auto MoveJointAllCos::executeRT()->int
{
    auto& param = std::any_cast<MoveJSParam&>(this->param());
    auto time = static_cast<int32_t>(param.time * 1000);
    auto totaltime = static_cast<int32_t>(param.timenum * time);
    static double begin_pjs[18] = { 0 };
    static double step_pjs[18] = { 0 };
    // 访问主站 //
    auto& cout = controller()->mout();

    if ((1 <= count()) && (count() <= time / 2))
    {
        // 获取当前起始点位置 //
        if (count() == 1)
        {
            for (int i = 0; i < 3; i++) {
                begin_pjs[i] = controller()->motionPool()[i].targetPos();
                step_pjs[i] = controller()->motionPool()[i].targetPos();
            }
        }
        for (int i = 0; i < 3; i++) {
            step_pjs[i] = begin_pjs[i] + param.amplitude * (1 - std::cos(2 * PI * count() / time)) / 2;
            controller()->motionPool().at(i).setTargetPos(step_pjs[i]);
        }

    }
    else if ((time / 2 < count()) && (count() <= totaltime - time / 2))
    {
        // 获取当前起始点位置 //
        if (count() == time / 2 + 1)
        {
            for (int i = 0; i < 3; i++) {
                begin_pjs[i] = controller()->motionPool()[i].targetPos();
                step_pjs[i] = controller()->motionPool()[i].targetPos();
            }
        }

        for (int i = 0; i < 3; i++) {
            step_pjs[i] = begin_pjs[i] + 2 * param.amplitude * (1 - std::cos(2 * PI * (count() - time / 2) / time)) / 2;
            controller()->motionPool().at(i).setTargetPos(step_pjs[i]);
        }
    }

    else if ((totaltime - time / 2 < count()) && (count() <= totaltime))
    {
        // 获取当前起始点位置 //
        if (count() == totaltime - time / 2 + 1)
        {
            for (int i = 0; i < 3; i++) {
                begin_pjs[i] = controller()->motionPool()[i].targetPos();
                step_pjs[i] = controller()->motionPool()[i].targetPos();
            }
        }
        for (int i = 0; i < 3; i++) {
            step_pjs[i] = begin_pjs[i] + param.amplitude * (1 - std::cos(2 * PI * (count() - totaltime + time / 2) / time)) / 2;
            controller()->motionPool().at(i).setTargetPos(step_pjs[i]);
        }
    }

    // 打印 //
    //if (count() % 100 == 0)
    //{
    //    cout << "pos" << ":" << controller()->motionAtAbs(0).actualPos() << "  ";
    //    cout << std::endl;
    //}

    // log //
    //auto& lout = controller()->lout();
    //lout << controller()->motionAtAbs(0).targetPos() << ",";
    //lout << std::endl;

    return totaltime - count();
}


auto MoveJointAllCos::collectNrt()->void {}
MoveJointAllCos::MoveJointAllCos(const std::string& name)
{
    aris::core::fromXmlString(command(),
        "<Command name=\"moveJAC\">"
        "	<GroupParam>"
        "		<Param name=\"amplitude\" default=\"15.0\" abbreviation=\"a\"/>"
        "		<Param name=\"time\" default=\"4.0\" abbreviation=\"t\"/>"
        "		<Param name=\"timenum\" default=\"5\" abbreviation=\"n\"/>"
        "	</GroupParam>"
        "</Command>");
}
MoveJointAllCos::~MoveJointAllCos() = default;




//---------------------hex 前进/后退--------------------//
//-x是正值是前进，-x是负值是后退，默认是前进-x=0.1
auto HexForward::prepareNrt()->void
{
    n_ = doubleParam("step_num");
    x_step_ = doubleParam("x_step");
    for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto HexForward::executeRT()->int
{
    static double begin_angle[18] = { 0 };
    if (count() == 1) {
//        for (int i = 0; i < 3; ++i) {
//            begin_angle[i] = controller()->motionPool()[i].targetPos();
////            mout() << begin_angle[0] << "\t" << begin_angle[1] << std::endl;
//        }
        this->master()->logFileRawName("hex_forward");
    }

    TCurve s1(4, 2);
    s1.getCurveParam();
    EllipseTrajectory e1(x_step_, 0.05, 0, s1);
    BodyPose body_s(0, 0, 0, s1);
    int ret = 0;
    ret = tripodPlan(n_, count() - 1, &e1, input_angle);

    double motor_angle[18] ={0};
    //for(int i = 0; i < 3 ;++i){
    //    motor_angle[i] = begin_angle[i] + input_angle[i];
    //}

    //输出电机角度，用于仿真测试
    {
        //log
//            for (int i = 0; i < 3; ++i) {
//                lout() << motor_angle[i] << "\t";
//            }
        for (int i = 0; i < 18; ++i) {
            lout() << input_angle[i] << "\t";
        }
        lout() << std::endl;

        //打印
//            for (int i = 0; i < 3; ++i) {
//                mout() << motor_angle[i] << "\t";
//            }
//            for (int i = 0; i < 3; ++i) {
//                mout() << input_angle[i+3] << "\t";
//            }

//            mout() << std::endl;
    }

    //输出身体和足端曲线，用于仿真测试
//        {
//            //log
//            for (int i = 0; i < 3; ++i) {
//                lout() << file_current_leg[i] << "\t";
//            }
//            lout() << file_current_body[3] << "\t" << file_current_body[7] << "\t" << file_current_body[11] << std::endl;



//            //打印
//            for (int i = 0; i < 3; ++i) {
//                mout() << file_current_leg[i] << "\t";
//            }
//            mout() << file_current_body[3] << "\t" << file_current_body[7] << "\t" << file_current_body[11] << std::endl;
//            mout() << std::endl;
//        }

    //给电机发送信号
        //for (int i = 0; i < 3; ++i) {
        //    controller()->motionPool()[i].setTargetPos(motor_angle[i]);
        //}
        //if (ret == 0){
        //    for (int i = 0; i < 3; ++i) {
        //        mout() << controller()->motionPool()[i].actualPos() <<std::endl;
        //    }
        //}
    return ret;
}

auto HexForward::collectNrt()->void {}
HexForward::HexForward(const std::string& name)
{
    aris::core::fromXmlString(command(),
        "<Command name=\"forward\">"
        "<GroupParam>"
        "<Param name=\"step_num\" default=\"2.0\" abbreviation=\"n\"/>"
        "<Param name=\"x_step\" default=\"0.1\" abbreviation=\"x\"/>"
        "</GroupParam>"
        "</Command>");
}
HexForward::~HexForward() = default;


    //---------------------hex 左移/右移--------------------//
    //-z是正值是右移，-z是负值是左移，默认是右移-z=0.1
    auto HexLateral::prepareNrt()->void
    {
        n_ = doubleParam("step_num");
        z_step_ = doubleParam("z_step");
        for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
    }
    auto HexLateral::executeRT()->int
    {
        static double begin_angle[18] = { 0 };
        if (count() == 1) {
            for (int i = 0; i < 3; ++i) {
                begin_angle[i] = controller()->motionPool()[i].targetPos();
            }
            this->master()->logFileRawName("hex_lateral");
        }

        TCurve s1(2, 1);
        s1.getCurveParam();
        EllipseTrajectory e1(0, 0.2, z_step_, s1);
        BodyPose body_s(0, 0, 0, s1);
        int ret = 0;
        ret = tripodPlan(n_, count() - 1, &e1, input_angle);
        double motor_angle[18] ={0};
        for(int i = 0; i < 3 ;++i){
            motor_angle[i] = begin_angle[i] + input_angle[i];
        }

        //输出电机角度，用于仿真测试
        {
            //log
            for (int i = 0; i < 3; ++i) {
                lout() << input_angle[i] << "\t";
            }
            lout() << std::endl;

            //打印
//            for (int i = 0; i < 3; ++i) {
//                mout() << input_angle[i] << "\t";
//            }
//            mout() << std::endl;
        }

//        //输出身体和足端曲线，用于仿真测试
//        {
//            //log
//            for (int i = 0; i < 3; ++i) {
//                lout() << file_current_leg[i] << "\t";
//            }
//            lout() << file_current_body[3] << "\t" << file_current_body[7] << "\t" << file_current_body[11] << std::endl;
//            lout() << std::endl;


//            //打印
//            for (int i = 0; i < 3; ++i) {
//                mout() << file_current_leg[i] << "\t";
//            }
//            mout() << file_current_body[3] << "\t" << file_current_body[7] << "\t" << file_current_body[11] << std::endl;
//            mout() << std::endl;
//        }

        //给电机发送信号
        for (int i = 0; i < 3; ++i) {
            controller()->motionPool()[i].setTargetPos(motor_angle[i]);
        }
        if (ret == 0){
            for (int i = 0; i < 3; ++i) {
                mout() << controller()->motionPool()[i].actualPos() <<std::endl;
            }
        }
        return ret;
    }


    auto HexLateral::collectNrt()->void {}
    HexLateral::HexLateral(const std::string& name)
    {
        aris::core::fromXmlString(command(),
            "<Command name=\"lateral\">"
            "<GroupParam>"
            "<Param name=\"step_num\" default=\"5.0\" abbreviation=\"n\"/>"
            "<Param name=\"z_step\" default=\"0.1\" abbreviation=\"z\"/>"
            "</GroupParam>"
            "</Command>");
    }
    HexLateral::~HexLateral() = default;


        //---------------------hex 左转/右转--------------------//
        //-y是正值是左转，-y是负值是右转，默认是右移-y=20
        auto HexTurn::prepareNrt()->void
        {
            n_ = doubleParam("step_num");
            turn_yaw_ = doubleParam("turn_yaw");
            for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
        }
        auto HexTurn::executeRT()->int
        {
            static double begin_angle[18] = { 0 };
            if (count() == 1) {
                for (int i = 0; i < 3; ++i) {
                    begin_angle[i] = controller()->motionPool()[i].targetPos();
                }
                this->master()->logFileRawName("hex_turn1");
            }

            TCurve s1(2, 1);
            s1.getCurveParam();
            EllipseTrajectory e1(0, 0.03, 0, s1);
            BodyPose body_s(0, turn_yaw_, 0, s1);
            int ret = 0;
            ret = turnPlanTripod(n_, count() - 1, &e1, &body_s, input_angle);
            double motor_angle[18] ={0};
            for(int i = 0; i < 3 ;++i){
                motor_angle[i] = begin_angle[i] + input_angle[i];
            }

            //输出电机角度，用于仿真测试
            {
                //log
                for (int i = 0; i < 3; ++i) {
                    lout() << motor_angle[i] << "\t";
                }
                lout() << std::endl;

//                //打印
//                for (int i = 0; i < 3; ++i) {
//                    mout() << input_angle[i] << "\t";
//                }
//                lout() << std::endl;
            }

            //输出身体和足端曲线，用于仿真测试
//            {
//                //log
//                for (int i = 0; i < 3; ++i) {
//                    lout() << file_current_leg[i] << "\t";
//                }
//                lout() << file_current_body[3] << "\t" << file_current_body[7] << "\t" << file_current_body[11] << std::endl;
//                lout() << std::endl;


//                //打印
//                for (int i = 0; i < 3; ++i) {
//                    mout() << file_current_leg[i] << "\t";
//                }
//                mout() << file_current_body[3] << "\t" << file_current_body[7] << "\t" << file_current_body[11] << std::endl;
//                mout() << std::endl;
//            }

            //给电机发送信号
            for (int i = 0; i < 3; ++i) {
                controller()->motionPool()[i].setTargetPos(motor_angle[i]);
            }
            if (ret == 0){
                for (int i = 0; i < 3; ++i) {
                    mout() << controller()->motionPool()[i].actualPos() <<std::endl;
                }
            }
            return ret;
        }


        auto HexTurn::collectNrt()->void {}
        HexTurn::HexTurn(const std::string& name)
        {
            aris::core::fromXmlString(command(),
                "<Command name=\"turn\">"
                "<GroupParam>"
                "<Param name=\"step_num\" default=\"5.0\" abbreviation=\"n\"/>"
                "<Param name=\"turn_yaw\" default=\"20\" abbreviation=\"y\"/>"
                "</GroupParam>"
                "</Command>");
        }
        HexTurn::~HexTurn() = default;



            //---------------------hex 四足步态--------------------//
            //-x是正值是前进，-x是负值是后退，默认是前进-x=0.1
            auto HexTetrapod::prepareNrt()->void
            {
                n_ = doubleParam("step_num");
                x_step_ = doubleParam("x_step");
                for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
            }
            auto HexTetrapod::executeRT()->int
            {
                static double begin_angle[18] = { 0 };
                if (count() == 1) {
                    for (int i = 0; i < 3; ++i) {
                        begin_angle[i] = controller()->motionPool()[i].targetPos();
                    }
                    this->master()->logFileRawName("hex_tetra");
                }


                TCurve s1(2, 1);
                s1.getCurveParam();
                EllipseTrajectory e1(x_step_, 0.03, 0, s1);
                BodyPose body_s(0, 0, 0, s1);
                int ret = 0;
                ret = tetrapodPlan(n_, count() - 1, &e1, input_angle);
                double motor_angle[18] ={0};
                for(int i = 0; i < 3 ;++i){
                    motor_angle[i] = begin_angle[i] + input_angle[i];
                }

                //输出电机角度，用于仿真测试
                {
                    //log
                    for (int i = 0; i < 3; ++i) {
                        lout() << input_angle[i] << "\t";
                    }
                    lout() << std::endl;

                    //打印
//                    for (int i = 0; i < 3; ++i) {
//                        mout() << input_angle[i] << "\t";
//                    }
//                    lout() << std::endl;
                }

                //输出身体和足端曲线，用于仿真测试
//                {
//                    //log
//                    for (int i = 0; i < 3; ++i) {
//                        lout() << file_current_leg[i] << "\t";
//                    }
//                    lout() << file_current_body[3] << "\t" << file_current_body[7] << "\t" << file_current_body[11] << std::endl;
//                    lout() << std::endl;


//                    //打印
//                    for (int i = 0; i < 3; ++i) {
//                        mout() << file_current_leg[i] << "\t";
//                    }
//                    mout() << file_current_body[3] << "\t" << file_current_body[7] << "\t" << file_current_body[11] << std::endl;
//                    mout() << std::endl;
//                }

                //给电机发送信号
                for (int i = 0; i < 3; ++i) {
                    controller()->motionPool()[i].setTargetPos(motor_angle[i]);
                }
                if (ret == 0){
                    for (int i = 0; i < 3; ++i) {
                        mout() << controller()->motionPool()[i].actualPos() <<std::endl;
                    }
                }
                return ret;
            }


            auto HexTetrapod::collectNrt()->void {}
            HexTetrapod::HexTetrapod(const std::string& name)
            {
                aris::core::fromXmlString(command(),
                    "<Command name=\"tetra\">"
                    "<GroupParam>"
                    "<Param name=\"step_num\" default=\"2.0\" abbreviation=\"n\"/>"
                    "<Param name=\"x_step\" default=\"0.1\" abbreviation=\"x\"/>"
                    "</GroupParam>"
                    "</Command>");
            }
            HexTetrapod::~HexTetrapod() = default;



//----------------------------------以下为仿真区域------------------------------------------//


            //---------------------------cpp和Adams调试------------------------//

            //前进
            auto HexDynamicForwardTest::prepareNrt()->void
        	{

        	}
        	auto HexDynamicForwardTest::executeRT()->int
        	{
                //数值解和实际解xyr相差一个负号
                //如果要输出cmd文件，则不能创建储存文件，需要注释掉
                //if (count() == 1)this->master()->logFileRawName("eeTraj");    
                //if (count() == 1)this->master()->logFileRawName("inputTraj");
                //if (count() == 1)this->master()->logFileRawName("invInput"); //反解计算结果储存文件，即解析解
                //if (count() == 1)this->master()->logFileRawName("numInput"); //数值解储存文件
                //if (count() == 1)this->master()->logFileRawName("leg1MotorPos1"); //拿到第1条腿的电机位置
                //if (count() == 1)this->master()->logFileRawName("leg1EndTraj1"); //拿到第1条腿的末端位置，要得到腿坐标系下的

                //a为给机器人缓冲落地的时间设置
        		int ret = 0,a=500;
                //末端为六个末端的三个坐标和身体的位姿矩阵 3*6+16=34
        		static double ee0[34];
        		double ee[34];

                //落地缓冲时间
                if (count() <= a)
                {
                    ret = 1;
                    if (count() == 1)
                    {
                        model()->getOutputPos(ee0); //初始位置
                        //s_vc好像是把ee0的数放到ee中，放34个数
                        aris::dynamic::s_vc(34, ee0, ee);
                    }
                    aris::dynamic::s_vc(34, ee0, ee);
                    model()->setOutputPos(ee);

                    //for (int i = 0; i < 34; i++) {
                    //    std::cout << ee[i] << "\t";
                    //}
                    //std::cout << std::endl;

                    //for (int i = 0; i < 19; ++i)
                    //    lout() << ee[i] << "\t";
                    //lout() << std::endl;






                    if (model()->inverseKinematics()) {
                        std::cout << "inverse failed " << std::endl;
                    } 
                    
                    model()->setTime(0.001 * count());
                }
                else
                {
                    TCurve s1(4, 2);
                    s1.getCurveParam();
                    EllipseTrajectory e1(0.01, 0.01, 0, s1);
                    BodyPose body_s(0, 0, 0, s1);


                    ret = tripodPlan(4, count() - 1-a, &e1, input_angle);
                    aris::dynamic::s_vc(16, file_current_body + 0, ee + 0);
                    //aris::dynamic::s_vc(18, foot_position_start_point + 0, ee + 16);

                    aris::dynamic::s_vc(18, file_current_leg + 0, ee + 16);


                    //for (int i = 0; i < 19; ++i)
                    //    lout() << ee[i] << "\t";
                    //lout() << std::endl;

                    //for (int i = 0; i < 34; i++) {
                    //    std::cout << ee[i] << std::endl;
                    //}



                    //
                    //std::cout << "here is else :" << std::endl;
                    //for (int i = 0; i < 34; i++) {
                    //    std::cout << ee[i] << "\t";
                    //}
                    //std::cout << std::endl;


                    //lout() << "here is else :" << std::endl;
                    //for (int i = 0; i < 34; ++i)
                    //    lout() << ee[i] << "\t";
                    //lout() << std::endl;
                    



                    //第一条腿电机的位置
                    //double leg1MotorPos[3] = { 0 };
                    //for (int i = 0; i < 3; ++i) {
                    //    lout() << input_angle[i] << "\t";
                    //}
                    //lout() << std::endl;

                    //解析解计算得到的输入的角度
                    //for (int i = 0; i < 18; ++i)
                    //    lout() << input_angle[i] << "\t";
                    //lout() << std::endl;

                    model()->setOutputPos(ee);
                    //model()->setinputpos(input_angle);
                    //if (model()->forwardkinematics()) {
                    //    std::cout << "forward failer!" << std::endl;
                    //}


                    if (model()->inverseKinematics())
                    {
                        
                        std::cout << "inverse failed!!!" << std::endl;
                        //for (int i = 0; i < 34; ++i) {
                        //    std::cout << ee[i] << std::endl;
                        //}
                        std::cout << "ret = " << ret << std::endl;
                    }
                   // 数值解计算得到的输入的角度
                    //double input[18];
                    //model()->getInputPos(input);
                    //for (int i = 0; i < 18; ++i)
                    //    lout() << input[i] << "\t";
                    //lout() << std::endl;

                    model()->setTime(0.001 * count());
                    std::cout << ret << std::endl;


                    if (ret == 0) std::cout << count() << std::endl;

                }
                //末端位置


                //for (int i = 0; i < 18; i++) {
                //    std::cout << ee[i + 16] << "\t";
                //    if (i % 4 == 3) {
                //        std::cout << std::endl;
                //    }
                //}


                return ret;

        	}
            HexDynamicForwardTest::HexDynamicForwardTest(const std::string& name)
            {
                aris::core::fromXmlString(command(),
                    "<Command name=\"hex_forward\"/>");
            }
            HexDynamicForwardTest::~HexDynamicForwardTest() = default;




            //单腿运动
            auto SingleLeg::prepareNrt()->void
            {

            }
            auto SingleLeg::executeRT()->int
            {
                //数值解和实际解xyr相差一个负号
                //如果要输出cmd文件，则不能创建储存文件，需要注释掉
                if (count() == 1)this->master()->logFileRawName("single");    


                //a为给机器人缓冲落地的时间设置
                int ret = 0, a = 100;
                //末端为六个末端的三个坐标和身体的位姿矩阵 3*6+16=34
                static double ee0[34];
                double ee[34];

                //落地缓冲时间
                if (count() <= a)
                {
                    ret = 1;
                    if (count() == 1)
                    {
                        model()->getOutputPos(ee0); //初始位置
                        //s_vc好像是把ee0的数放到ee中，放34个数
                        aris::dynamic::s_vc(34, ee0, ee);
                    }
                    aris::dynamic::s_vc(34, ee0, ee);
                    model()->setOutputPos(ee);


                    if (model()->inverseKinematics()) {
                        std::cout << "inverse failed " << std::endl;
                    }

                    model()->setTime(0.001 * count());
                }
                else
                {



                    double body_pos[16] = {1,0,0,0,
                                            0,1,0,0,
                                            0,0,1,0,
                                            0,0,0,1};

                    double l = 0.0463672;
                    
                    double coeff = (count()-a) / 100.0;
                    double theta = 0 / 100.0 * coeff;


                    if (coeff == 1) {
                        ret = 0;
                    }
                    else {
                        ret = 1;
                    }

                    //double leg1[3] = { 0.05+l * sin(theta),-0.0025-l * cos(theta),0.026 };
                    double leg1[3] = { 0.05, -0.0488072, 0.026 };
                    std::cout << leg1[0] << "\t" << leg1[1] << "\t" << leg1[2] << std::endl;







                    //这些坐标都需要是在地面坐标系下的坐标值
                    aris::dynamic::s_vc(16, body_pos + 0, ee + 0);
                    aris::dynamic::s_vc(18, foot_position_start_point + 0, ee + 16);
                    aris::dynamic::s_vc(3, leg1 + 0, ee + 16);







                    //第一条腿电机的位置
                    //double leg1MotorPos[3] = { 0 };
                    //for (int i = 0; i < 3; ++i) {
                    //    lout() << input_angle[i] << "\t";
                    //}
                    //lout() << std::endl;

                    //解析解计算得到的输入的角度
                    //for (int i = 0; i < 18; ++i)
                    //    lout() << input_angle[i] << "\t";
                    //lout() << std::endl;

                    model()->setOutputPos(ee);
                    //model()->setinputpos(input_angle);
                    //if (model()->forwardkinematics()) {
                    //    std::cout << "forward failer!" << std::endl;
                    //}


                    if (model()->inverseKinematics())
                    {

                        std::cout << "inverse failed!!!" << std::endl;
                        //for (int i = 0; i < 34; ++i) {
                        //    std::cout << ee[i] << std::endl;
                        //}
                        std::cout << "ret = " << ret << std::endl;
                    }
                    // 数值解计算得到的输入的角度
                     double input[18];
                     model()->getInputPos(input);
                     for (int i = 0; i < 18; ++i)
                         lout() << input[i] << "\t";
                     lout() << std::endl;

                    model()->setTime(0.001 * count());



                    if (ret == 0) std::cout << count() << std::endl;

                }
                ////末端位置
                //for (int i = 0; i < 34; ++i)
                //    lout() << ee[i] << "\t";
                //lout() << std::endl;
                return ret;

            }
            SingleLeg::SingleLeg(const std::string& name)
            {
                aris::core::fromXmlString(command(),
                    "<Command name=\"single\"/>");
            }
            SingleLeg::~SingleLeg() = default;


            //单腿运动
            auto MoveBody::prepareNrt()->void
            {

            }
            auto MoveBody::executeRT()->int
            {
                //数值解和实际解xyr相差一个负号
                //如果要输出cmd文件，则不能创建储存文件，需要注释掉
                if (count() == 1)this->master()->logFileRawName("single");


                //a为给机器人缓冲落地的时间设置
                int ret = 0, a = 100;
                //末端为六个末端的三个坐标和身体的位姿矩阵 3*6+16=34
                static double ee0[34];
                double ee[34];

                //落地缓冲时间
                if (count() <= a)
                {
                    ret = 1;
                    if (count() == 1)
                    {
                        model()->getOutputPos(ee0); //初始位置
                        //s_vc好像是把ee0的数放到ee中，放34个数
                        aris::dynamic::s_vc(34, ee0, ee);
                    }
                    aris::dynamic::s_vc(34, ee0, ee);
                    model()->setOutputPos(ee);


                    if (model()->inverseKinematics()) {
                        std::cout << "inverse failed " << std::endl;
                    }

                    model()->setTime(0.001 * count());
                }
                else
                {



                    double body_pos[16] = { 1,0,0,0,
                                            0,1,0,0,
                                            0,0,1,0,
                                            0,0,0,1 };

                    double l = 0.0463672;

                    double coeff = (count() - a) / 100.0;
                    double theta = 0 / 100.0 * coeff;


                    if (coeff == 1) {
                        ret = 0;
                    }
                    else {
                        ret = 1;
                    }

                    //double leg1[3] = { 0.05+l * sin(theta),-0.0025-l * cos(theta),0.026 };
                    double leg1[3] = { 0.05, -0.0488072, 0.026 };
                    std::cout << leg1[0] << "\t" << leg1[1] << "\t" << leg1[2] << std::endl;







                    //这些坐标都需要是在地面坐标系下的坐标值
                    aris::dynamic::s_vc(16, body_pos + 0, ee + 0);
                    aris::dynamic::s_vc(18, foot_position_start_point + 0, ee + 16);
                    aris::dynamic::s_vc(3, leg1 + 0, ee + 16);







                    //第一条腿电机的位置
                    //double leg1MotorPos[3] = { 0 };
                    //for (int i = 0; i < 3; ++i) {
                    //    lout() << input_angle[i] << "\t";
                    //}
                    //lout() << std::endl;

                    //解析解计算得到的输入的角度
                    //for (int i = 0; i < 18; ++i)
                    //    lout() << input_angle[i] << "\t";
                    //lout() << std::endl;

                    model()->setOutputPos(ee);
                    //model()->setinputpos(input_angle);
                    //if (model()->forwardkinematics()) {
                    //    std::cout << "forward failer!" << std::endl;
                    //}


                    if (model()->inverseKinematics())
                    {

                        std::cout << "inverse failed!!!" << std::endl;
                        //for (int i = 0; i < 34; ++i) {
                        //    std::cout << ee[i] << std::endl;
                        //}
                        std::cout << "ret = " << ret << std::endl;
                    }
                    // 数值解计算得到的输入的角度
                    double input[18];
                    model()->getInputPos(input);
                    for (int i = 0; i < 18; ++i)
                        lout() << input[i] << "\t";
                    lout() << std::endl;

                    model()->setTime(0.001 * count());



                    if (ret == 0) std::cout << count() << std::endl;

                }
                ////末端位置
                //for (int i = 0; i < 34; ++i)
                //    lout() << ee[i] << "\t";
                //lout() << std::endl;
                return ret;

            }
            MoveBody::MoveBody(const std::string& name)
            {
                aris::core::fromXmlString(command(),
                    "<Command name=\"single\"/>");
            }
            MoveBody::~MoveBody() = default;




            //后退
            auto HexDynamicBackTest::prepareNrt()->void
            {

            }
            auto HexDynamicBackTest::executeRT()->int
            {
                //如果要输出cmd文件，则不能创建储存文件，需要注释掉
                //if (count() == 1)this->master()->logFileRawName("eeTraj");    

                //a为给机器人缓冲落地的时间设置
                int ret = 0, a = 500;
                //末端为六个末端的三个坐标和身体的位姿矩阵 3*6+16=34
                static double ee0[34];
                double ee[34];

                //落地缓冲时间
                if (count() <= a)
                {
                    ret = 1;
                    if (count() == 1)
                    {
                        model()->getOutputPos(ee0); //初始位置
                        //s_vc好像是把ee0的数放到ee中，放34个数
                        aris::dynamic::s_vc(34, ee0, ee);
                    }
                    aris::dynamic::s_vc(34, ee0, ee);
                    model()->setOutputPos(ee);


                    if (model()->inverseKinematics()) std::cout << "inverse failed " << std::endl;

                    model()->setTime(0.001 * count());
                }
                else
                {
                    TCurve s1(4, 2);
                    s1.getCurveParam();
                    EllipseTrajectory e1(-0.1, 0.05, 0, s1);
                    BodyPose body_s(0, 0, 0, s1);


                    ret = tripodPlan(2, count() - 1 - a, &e1, input_angle);
                    aris::dynamic::s_vc(16, file_current_body + 0, ee + 0);
                    aris::dynamic::s_vc(18, file_current_leg + 0, ee + 16);


                    model()->setOutputPos(ee);



                    if (model()->inverseKinematics())
                    {

                        std::cout << "inverse failed!!!" << std::endl;
                        //for (int i = 0; i < 34; ++i) {
                        //    std::cout << ee[i] << std::endl;
                        //}
                        std::cout << "ret = " << ret << std::endl;
                    }

                    model()->setTime(0.001 * count());


                    if (ret == 0) std::cout << count() << std::endl;

                }
                return ret;
            }
            HexDynamicBackTest::HexDynamicBackTest(const std::string& name)
            {
                aris::core::fromXmlString(command(),
                    "<Command name=\"hex_back\"/>");
            }
            HexDynamicBackTest::~HexDynamicBackTest() = default;

            //右移
            auto HexDynamicRightTest::prepareNrt()->void
            {

            }
            auto HexDynamicRightTest::executeRT()->int
            {
                //如果要输出cmd文件，则不能创建储存文件，需要注释掉
                //if (count() == 1)this->master()->logFileRawName("eeTraj");    
                //if (count() == 1)this->master()->logFileRawName("MotorPos"); //拿到第1条腿的电机位置
                //if (count() == 1)this->master()->logFileRawName("leg1EndTraj"); //拿到第1条腿的末端位置，要得到腿坐标系下的

                //a为给机器人缓冲落地的时间设置
                int ret = 0, a = 500;
                //末端为六个末端的三个坐标和身体的位姿矩阵 3*6+16=34
                static double ee0[34];
                double ee[34];

                //落地缓冲时间
                if (count() <= a)
                {
                    ret = 1;
                    if (count() == 1)
                    {
                        model()->getOutputPos(ee0); //初始位置
                        //s_vc好像是把ee0的数放到ee中，放34个数
                        aris::dynamic::s_vc(34, ee0, ee);
                    }
                    aris::dynamic::s_vc(34, ee0, ee);
                    model()->setOutputPos(ee);


                    if (model()->inverseKinematics()) std::cout << "inverse failed " << std::endl;

                    model()->setTime(0.001 * count());
                }
                else
                {
                    TCurve s1(4, 2);
                    s1.getCurveParam();
                    EllipseTrajectory e1(0, 0.01, 0.01, s1);
                    BodyPose body_s(0, 0, 0, s1);


                    ret = tripodPlan(4, count() - 1 - a, &e1, input_angle);
                    aris::dynamic::s_vc(16, file_current_body + 0, ee + 0);
                    aris::dynamic::s_vc(18, file_current_leg + 0, ee + 16);


                    model()->setOutputPos(ee);


                    //for (int i = 0; i < 3; ++i) {
                    //    lout() << input_angle[i] << "\t";
                    //}
                    //lout() << std::endl;


                    //末端位置
                    //for (int i = 0; i < 34; ++i)
                    //    lout() << ee[i] << "\t";
                    //lout() << std::endl;



                    if (model()->inverseKinematics())
                    {

                        std::cout << "inverse failed!!!" << std::endl;
                        //for (int i = 0; i < 34; ++i) {
                        //    std::cout << ee[i] << std::endl;
                        //}
                        std::cout << "ret = " << ret << std::endl;
                    }


                    //double input[18];
                    //model()->getInputPos(input);
                    //for (int i = 0; i < 18; ++i)
                    //    lout() << input[i] << "\t";
                    //lout() << std::endl;

                    model()->setTime(0.001 * count());


                    if (ret == 0) std::cout << count() << std::endl;

                }
                return ret;
               
            }
            HexDynamicRightTest::HexDynamicRightTest(const std::string& name)
            {
                aris::core::fromXmlString(command(),
                    "<Command name=\"hex_right\"/>");
            }
            HexDynamicRightTest::~HexDynamicRightTest() = default;



            //左移
            auto HexDynamicLeftTest::prepareNrt()->void
            {

            }
            auto HexDynamicLeftTest::executeRT()->int
            {
                //如果要输出cmd文件，则不能创建储存文件，需要注释掉
               //if (count() == 1)this->master()->logFileRawName("eeTraj");    

               //a为给机器人缓冲落地的时间设置
                int ret = 0, a = 500;
                //末端为六个末端的三个坐标和身体的位姿矩阵 3*6+16=34
                static double ee0[34];
                double ee[34];

                //落地缓冲时间
                if (count() <= a)
                {
                    ret = 1;
                    if (count() == 1)
                    {
                        model()->getOutputPos(ee0); //初始位置
                        //s_vc好像是把ee0的数放到ee中，放34个数
                        aris::dynamic::s_vc(34, ee0, ee);
                    }
                    aris::dynamic::s_vc(34, ee0, ee);
                    model()->setOutputPos(ee);


                    if (model()->inverseKinematics()) std::cout << "inverse failed " << std::endl;

                    model()->setTime(0.001 * count());
                }
                else
                {
                    TCurve s1(4, 2);
                    s1.getCurveParam();
                    EllipseTrajectory e1(0, 0.05, -0.1, s1);
                    BodyPose body_s(0, 0, 0, s1);


                    ret = tripodPlan(2, count() - 1 - a, &e1, input_angle);
                    aris::dynamic::s_vc(16, file_current_body + 0, ee + 0);
                    aris::dynamic::s_vc(18, file_current_leg + 0, ee + 16);


                    model()->setOutputPos(ee);



                    if (model()->inverseKinematics())
                    {

                        std::cout << "inverse failed!!!" << std::endl;
                        //for (int i = 0; i < 34; ++i) {
                        //    std::cout << ee[i] << std::endl;
                        //}
                        std::cout << "ret = " << ret << std::endl;
                    }

                    model()->setTime(0.001 * count());


                    if (ret == 0) std::cout << count() << std::endl;

                }
                return ret;
            }
            HexDynamicLeftTest::HexDynamicLeftTest(const std::string& name)
            {
                aris::core::fromXmlString(command(),
                    "<Command name=\"hex_left\"/>");
            }
            HexDynamicLeftTest::~HexDynamicLeftTest() = default;


            //右转
            auto HexDynamicTurnRightTest::prepareNrt()->void
            {

            }
            auto HexDynamicTurnRightTest::executeRT()->int
            {

                //如果要输出cmd文件，则不能创建储存文件，需要注释掉
                //if (count() == 1)this->master()->logFileRawName("eeTraj");
                //if (count() == 1)this->master()->logFileRawName("motInput");  
                //if (count() == 1)this->master()->logFileRawName("leg1MotorPos2"); //拿到第1条腿的电机位置
                //if (count() == 1)this->master()->logFileRawName("leg1EndTraj2"); //拿到第1条腿的末端位置，要得到腿坐标系下的

                //a为给机器人缓冲落地的时间设置
                int ret = 0, a = 500;
                //末端为六个末端的三个坐标和身体的位姿矩阵 3*6+16=34
                static double ee0[34];
                double ee[34];

                //落地缓冲时间
                if (count() <= a)
                {
                    ret = 1;
                    if (count() == 1)
                    {
                        model()->getOutputPos(ee0); //初始位置
                        //s_vc好像是把ee0的数放到ee中，放34个数
                        aris::dynamic::s_vc(34, ee0, ee);
                    }
                    aris::dynamic::s_vc(34, ee0, ee);
                    model()->setOutputPos(ee);


                    if (model()->inverseKinematics()) std::cout << "inverse failed " << std::endl;

                    model()->setTime(0.001 * count());
                }
                else
                {
                    TCurve s1(4, 2);
                    s1.getCurveParam();
                    EllipseTrajectory e1(0, 0.001, 0, s1);
                    BodyPose body_s(0,10, 0, s1); //目前看来就是这个正负号影响步长的改变


                    ret = turnPlanTripod(2, count() - 1 - a, &e1, &body_s, input_angle);
                    aris::dynamic::s_vc(16, file_current_body + 0, ee + 0);
                    aris::dynamic::s_vc(18, file_current_leg + 0, ee + 16);


                    model()->setOutputPos(ee);

                    //末端位置
                    //for (int i = 0; i < 34; ++i)
                    //    lout() << ee[i] << "\t";
                    //lout() << std::endl;


                    //for (int i = 0; i < 3; ++i) {
                    //    lout() << input_angle[i] << "\t";
                    //}
                    //lout() << std::endl;



                    if (model()->inverseKinematics())
                    {

                        std::cout << "inverse failed!!!" << std::endl;
                        //for (int i = 0; i < 34; ++i) {
                        //    std::cout << ee[i] << std::endl;
                        //}
                        std::cout << "ret = " << ret << std::endl;
                    }

                    // 数值解计算得到的输入的角度
                    //double input[18];
                    //model()->getInputPos(input);
                    //for (int i = 0; i < 18; ++i)
                    //    lout() << input[i] << "\t";
                    //lout() << std::endl;

                    model()->setTime(0.001 * count());


                    if (ret == 0) std::cout << count() << std::endl;

                }

                //for (int i = 0; i < 34; ++i)
                //    lout() << ee[i] << "\t";
                //lout() << std::endl;
                return ret;

            }
            HexDynamicTurnRightTest::HexDynamicTurnRightTest(const std::string& name)
            {
                aris::core::fromXmlString(command(),
                    "<Command name=\"hex_turn_right\"/>");
            }
            HexDynamicTurnRightTest::~HexDynamicTurnRightTest() = default;

            //左转
            auto HexDynamicTurnLeftTest::prepareNrt()->void
            {

            }
            auto HexDynamicTurnLeftTest::executeRT()->int
            {
                //如果要输出cmd文件，则不能创建储存文件，需要注释掉
                //if (count() == 1)this->master()->logFileRawName("eeTraj");    

                //a为给机器人缓冲落地的时间设置
                int ret = 0, a = 500;
                //末端为六个末端的三个坐标和身体的位姿矩阵 3*6+16=34
                static double ee0[19];
                double ee[19];

                //落地缓冲时间
                if (count() <= a)
                {
                    ret = 1;
                    if (count() == 1)
                    {
                        model()->getOutputPos(ee0); //初始位置
                        //s_vc好像是把ee0的数放到ee中，放34个数
                        aris::dynamic::s_vc(19, ee0, ee);
                    }
                    aris::dynamic::s_vc(19, ee0, ee);
                    model()->setOutputPos(ee);


                    if (model()->inverseKinematics()) std::cout << "inverse failed " << std::endl;

                    model()->setTime(0.001 * count());
                }
                else
                {
                    TCurve s1(4, 2);
                    s1.getCurveParam();
                    EllipseTrajectory e1(0, 0.005, 0, s1);
                    BodyPose body_s(0, 20, 0, s1);


                    ret = turnPlanTripod(2, count() - 1 - a, &e1, &body_s, input_angle);
                    aris::dynamic::s_vc(16, file_current_body + 0, ee + 0);
                    aris::dynamic::s_vc(3, file_current_leg + 0, ee + 16);


                    model()->setOutputPos(ee);



                    if (model()->inverseKinematics())
                    {

                        std::cout << "inverse failed!!!" << std::endl;
                        //for (int i = 0; i < 34; ++i) {
                        //    std::cout << ee[i] << std::endl;
                        //}
                        std::cout << "ret = " << ret << std::endl;
                    }

                    model()->setTime(0.001 * count());


                    if (ret == 0) std::cout << count() << std::endl;

                }
                return ret;

            }
            HexDynamicTurnLeftTest::HexDynamicTurnLeftTest(const std::string& name)
            {
                aris::core::fromXmlString(command(),
                    "<Command name=\"hex_turn_right\"/>");
            }
            HexDynamicTurnLeftTest::~HexDynamicTurnLeftTest() = default;

            //四足步态
            auto HexDynamicTetrapodTest::prepareNrt()->void
            {

            }
            auto HexDynamicTetrapodTest::executeRT()->int
            {

                //如果要输出cmd文件，则不能创建储存文件，需要注释掉
                if (count() == 1)this->master()->logFileRawName("eeTraj");    

                //a为给机器人缓冲落地的时间设置
                int ret = 0, a = 500;
                //末端为六个末端的三个坐标和身体的位姿矩阵 3*6+16=34
                static double ee0[34];
                double ee[34];

                //落地缓冲时间
                if (count() <= a)
                {
                    ret = 1;
                    if (count() == 1)
                    {
                        model()->getOutputPos(ee0); //初始位置
                        //s_vc好像是把ee0的数放到ee中，放34个数
                        aris::dynamic::s_vc(34, ee0, ee);
                    }
                    aris::dynamic::s_vc(34, ee0, ee);
                    model()->setOutputPos(ee);


                    if (model()->inverseKinematics()) std::cout << "inverse failed " << std::endl;

                    model()->setTime(0.001 * count());
                }
                else
                {
                    TCurve s1(4, 2);
                    s1.getCurveParam();
                    EllipseTrajectory e1(0.1, 0.05, 0, s1);
                    BodyPose body_s(0, 0, 0, s1);


                    ret = tetrapodPlan(3, count() - 1 - a, &e1, input_angle);
                    aris::dynamic::s_vc(16, file_current_body + 0, ee + 0);
                    aris::dynamic::s_vc(18, file_current_leg + 0, ee + 16);


                    model()->setOutputPos(ee);

                    ////末端位置
                    for (int i = 0; i < 34; ++i)
                        lout() << ee[i] << "\t";
                    lout() << std::endl;

                    if (model()->inverseKinematics())
                    {

                        std::cout << "inverse failed!!!" << std::endl;
                        //for (int i = 0; i < 34; ++i) {
                        //    std::cout << ee[i] << std::endl;
                        //}
                        std::cout << "ret = " << ret << std::endl;
                    }

                    model()->setTime(0.001 * count());


                    if (ret == 0) std::cout << count() << std::endl;

                }
                return ret;

            }
            HexDynamicTetrapodTest::HexDynamicTetrapodTest(const std::string& name)
            {
                aris::core::fromXmlString(command(),
                    "<Command name=\"hex_tetrapod\"/>");
            }
            HexDynamicTetrapodTest::~HexDynamicTetrapodTest() = default;






            auto createModelHexapod()->std::unique_ptr<aris::dynamic::Model>
            {
                std::unique_ptr<aris::dynamic::Model> hex = std::make_unique<aris::dynamic::Model>();
                // set gravity //
                const double gravity[6]{ 0.0,-9.8,0.0,0.0,0.0,0.0 };
                hex->environment().setGravity(gravity);

                //define joint pos //
                //此处都是在初始位置下测量,坐标系朝向与Adams一致
                const double leg1_pe[8][3]{
                    {0.055,      -0.003,      0.0}, //虎克铰 x轴
                    {0.055,      -0.003,      0.0}, //虎克铰 z轴
                    {0.0690950,  -0.0112035,  0.0132935}, //J1 75&-45_20
                    {0.0675950,  -0.0284562,  0.0031769}, //J2 -45_20&45_20_end
                    {0.0690950,  -0.0458437,  0.0130599}, //J3 45_20_end&75_end
                    {0.0690950,  -0.0425026,  -0.0110603}, //J4 -45_20_end&75_end
                    {0.0705950,  -0.0284562,  0.0031769}, //J5 45_20&-45_20_end
                    {0.0690950,  -0.0142190,  -0.0108696} //J6 75&45_20  
                };

                const double leg2_pe[8][3]{
                    {0.0275,     -0.003, 0.0476314},
                    {0.0275,     -0.003, 0.0476314},
                    {0.023035, -0.0112035, 0.0664848},
                    {0.0310462, -0.0284562, 0.0601274},
                    {0.0232373, -0.0458437,  0.066368},
                    {0.044126, -0.0425026, 0.0543079},
                    {0.0325462, -0.0284562, 0.0627255},
                    {0.0439608,  -0.014219, 0.0544032}
                };

                const double leg3_pe[8][3]{
                    {-0.0275,     -0.003, 0.0476314},
                    {-0.0275,     -0.003, 0.0476314},
                    {-0.04606, -0.0112035, 0.0531913},
                    {-0.0365488, -0.0284562, 0.0569505},
                    {-0.0458577, -0.0458437, 0.0533081},
                    {-0.024969, -0.0425026, 0.0653682},
                    {-0.0380488, -0.0284562, 0.0595486},
                    {-0.0251342,  -0.014219, 0.0652728}
                };

                const double leg4_pe[8][3]{
                    {-0.055,     -0.003, 0},
                    {-0.055,     -0.003, 0},
                    {-0.069095, -0.0112035,  -0.0132935},
                    {-0.067595, -0.0284562,  -0.0031769},
                    {-0.069095, -0.0458437,  -0.0130599},
                    {-0.069095, -0.0425026,   0.0110603},
                    {-0.070595, -0.0284562,  -0.0031769},
                    {-0.069095,  -0.014219,   0.0108696}
                };

                const double leg5_pe[8][3]{
                    {-0.0275,     -0.003, -0.0476314},
                    {-0.0275,     -0.003, -0.0476314},
                    {-0.023035, -0.0112035, -0.0664848},
                    {-0.0310462, -0.0284562, -0.0601274},
                    {-0.0232373, -0.0458437,  -0.066368},
                    {-0.044126, -0.0425026, -0.0543079},
                    {-0.0325462, -0.0284562, -0.0627255},
                    {-0.0439608,  -0.014219, -0.0544032}
                };

                const double leg6_pe[8][3]{
                    {0.0275,     -0.003, -0.0476314},
                    {0.0275,     -0.003, -0.0476314},
                    {0.04606, -0.0112035, -0.0531913},
                    {0.0365488, -0.0284562, -0.0569505},
                    {0.0458577, -0.0458437, -0.0533081},
                    {0.024969, -0.0425026, -0.0653682},
                    {0.0380488, -0.0284562, -0.0595486},
                    {0.0251342,  -0.014219, -0.0652728}
                };

                //define ee pos  六条腿的末端//
                const double ee_pos[6][6]
                {
                    {0.055,         -0.0513908,        0.0,		0.0,    0.0,    0.0},
                    {0.0275,        -0.0513908,        0.0476314,		0.0,    0.0,    0.0},
                    {-0.0275,       -0.0513908,        0.0476314,	    0.0,    0.0,    0.0},
                    {-0.055,        -0.0513908,        0.0,	    0.0,    0.0,    0.0},
                    {-0.0275,       -0.0513908,        -0.0476314,		0.0,    0.0,    0.0},
                    {0.0275,        -0.0513908,        -0.0476314,	    0.0,    0.0,    0.0},
                };


                //这部分物理参数在最新的模型中还暂未修改
                //iv:  10x1 惯量矩阵向量[m, cx, cy, cz, Ixx, Iyy, Izz, Ixy, Ixz, Iyz]
                const double body_iv[10]{ 22.99,0,0,0,0.792149,0.792441,1.201794,0.000323,0.000351,0.000161 };
                //每条腿都在自己坐标系下，故惯性张量一样，不作区分，单腿重4.208793kg  总重35.872758kg，还有部分组件没有考虑上，估计在50kg左右
                const double leg_75[10]{5.185207,0,0,0,0.067462,0.016493,0.080198,0.013121,0.000005,0.000012};
                const double leg_m45_20[10]{0.138307,0,0,0,0.000286,0.000021,0.000302,0.000025,0.00,0.00};
                const double leg_45_20_end[10]{0.206814,0,0,0,0.000519,0.000433,0.000674,0.000315,0.0,0.000001};
                const double leg_75_end[10]{0.566247,0,0,0,0.012401,0.000164,0.012383,0.000681,0.000000,0.000001};
                const double leg_m45_20_end[10]{0.123833,0,0,0,0.000292,0.000226,0.000495,0.000241,0.0,0.0};
                const double leg_45_20[10]{0.117524,0,0,0,0.000175,0.000033,0.000197,0.000043,0.0,0.0};

                const double leg_z[10]{ 0.118757,0,0,0,0.000123,0.000042,0.000094,0.000004,0.0,0.0 };
                //const double shortest_bar_iv[10]{0.030776,0,0,0,0.000007,0.000009,0.000007,0,0,0};
                //const double x_screw_iv[10]{0.101926,0,0,0,0.000006,0.000232,0.000234,0.000006,0.0,0.0};

                //add part //
                auto& body = hex->partPool().add<aris::dynamic::Part>("BODY", body_iv);
                //leg1

                auto& leg1_z = hex->partPool().add<aris::dynamic::Part>("leg1_z", leg_z);
                auto& leg1_75 = hex->partPool().add<aris::dynamic::Part>("leg1_75", leg_75);
                auto& leg1_m45_20 = hex->partPool().add<aris::dynamic::Part>("leg1_m45_20", leg_m45_20);
                auto& leg1_45_20_end = hex->partPool().add<aris::dynamic::Part>("leg1_45_20_end", leg_45_20_end);
                auto& leg1_75_end = hex->partPool().add<aris::dynamic::Part>("leg1_75_end", leg_75_end);
                auto& leg1_m45_20_end = hex->partPool().add<aris::dynamic::Part>("leg1_m45_20_end", leg_m45_20_end);
                auto& leg1_45_20 = hex->partPool().add<aris::dynamic::Part>("leg1_45_20", leg_45_20);

                //leg2

                auto& leg2_z = hex->partPool().add<aris::dynamic::Part>("leg2_z", leg_z);
                auto& leg2_75 = hex->partPool().add<aris::dynamic::Part>("leg2_75", leg_75);
                auto& leg2_m45_20 = hex->partPool().add<aris::dynamic::Part>("leg2_m45_20", leg_m45_20);
                auto& leg2_45_20_end = hex->partPool().add<aris::dynamic::Part>("leg2_45_20_end", leg_45_20_end);
                auto& leg2_75_end = hex->partPool().add<aris::dynamic::Part>("leg2_75_end", leg_75_end);
                auto& leg2_m45_20_end = hex->partPool().add<aris::dynamic::Part>("leg2_m45_20_end", leg_m45_20_end);
                auto& leg2_45_20 = hex->partPool().add<aris::dynamic::Part>("leg2_45_20", leg_45_20);

                //leg3

                auto& leg3_z = hex->partPool().add<aris::dynamic::Part>("leg3_z", leg_z);
                auto& leg3_75 = hex->partPool().add<aris::dynamic::Part>("leg3_75", leg_75);
                auto& leg3_m45_20 = hex->partPool().add<aris::dynamic::Part>("leg3_m45_20", leg_m45_20);
                auto& leg3_45_20_end = hex->partPool().add<aris::dynamic::Part>("leg3_45_20_end", leg_45_20_end);
                auto& leg3_75_end = hex->partPool().add<aris::dynamic::Part>("leg3_75_end", leg_75_end);
                auto& leg3_m45_20_end = hex->partPool().add<aris::dynamic::Part>("leg3_m45_20_end", leg_m45_20_end);
                auto& leg3_45_20 = hex->partPool().add<aris::dynamic::Part>("leg3_45_20", leg_45_20);

                //leg4

                auto& leg4_z = hex->partPool().add<aris::dynamic::Part>("leg4_z", leg_z);
                auto& leg4_75 = hex->partPool().add<aris::dynamic::Part>("leg4_75", leg_75);
                auto& leg4_m45_20 = hex->partPool().add<aris::dynamic::Part>("leg4_m45_20", leg_m45_20);
                auto& leg4_45_20_end = hex->partPool().add<aris::dynamic::Part>("leg4_45_20_end", leg_45_20_end);
                auto& leg4_75_end = hex->partPool().add<aris::dynamic::Part>("leg4_75_end", leg_75_end);
                auto& leg4_m45_20_end = hex->partPool().add<aris::dynamic::Part>("leg4_m45_20_end", leg_m45_20_end);
                auto& leg4_45_20 = hex->partPool().add<aris::dynamic::Part>("leg4_45_20", leg_45_20);

                //leg5

                auto& leg5_z = hex->partPool().add<aris::dynamic::Part>("leg5_z", leg_z);
                auto& leg5_75 = hex->partPool().add<aris::dynamic::Part>("leg5_75", leg_75);
                auto& leg5_m45_20 = hex->partPool().add<aris::dynamic::Part>("leg5_m45_20", leg_m45_20);
                auto& leg5_45_20_end = hex->partPool().add<aris::dynamic::Part>("leg5_45_20_end", leg_45_20_end);
                auto& leg5_75_end = hex->partPool().add<aris::dynamic::Part>("leg5_75_end", leg_75_end);
                auto& leg5_m45_20_end = hex->partPool().add<aris::dynamic::Part>("leg5_m45_20_end", leg_m45_20_end);
                auto& leg5_45_20 = hex->partPool().add<aris::dynamic::Part>("leg5_45_20", leg_45_20);

                //leg6

                auto& leg6_z = hex->partPool().add<aris::dynamic::Part>("leg6_z", leg_z);
                auto& leg6_75 = hex->partPool().add<aris::dynamic::Part>("leg6_75", leg_75);
                auto& leg6_m45_20 = hex->partPool().add<aris::dynamic::Part>("leg6_m45_20", leg_m45_20);
                auto& leg6_45_20_end = hex->partPool().add<aris::dynamic::Part>("leg6_45_20_end", leg_45_20_end);
                auto& leg6_75_end = hex->partPool().add<aris::dynamic::Part>("leg6_75_end", leg_75_end);
                auto& leg6_m45_20_end = hex->partPool().add<aris::dynamic::Part>("leg6_m45_20_end", leg_m45_20_end);
                auto& leg6_45_20 = hex->partPool().add<aris::dynamic::Part>("leg6_45_20", leg_45_20);



                //add geometry //
                //hex->ground().geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\ground.x_t");
                body.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\bianbao\\x_t_hex_bend\\body.x_t");
                // leg1

                leg1_z.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\bianbao\\x_t_hex_bend\\leg1_z.x_t");
                leg1_75.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\bianbao\\x_t_hex_bend\\leg1_75.x_t");
                leg1_m45_20.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\bianbao\\x_t_hex_bend\\leg1_-45_20.x_t");
                leg1_45_20_end.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\bianbao\\x_t_hex_bend\\leg1_45_20_end.x_t");
                leg1_75_end.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\bianbao\\x_t_hex_bend\\leg1_75_end.x_t");
                leg1_m45_20_end.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\bianbao\\x_t_hex_bend\\leg1_-45_20_end.x_t");
                leg1_45_20.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\bianbao\\x_t_hex_bend\\leg1_45_20.x_t");
                
                // leg2

                leg2_z.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\bianbao\\x_t_hex_bend\\leg2_z.x_t");
                leg2_75.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\bianbao\\x_t_hex_bend\\leg2_75.x_t");
                leg2_m45_20.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\bianbao\\x_t_hex_bend\\leg2_-45_20.x_t");
                leg2_45_20_end.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\bianbao\\x_t_hex_bend\\leg2_45_20_end.x_t");
                leg2_75_end.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\bianbao\\x_t_hex_bend\\leg2_75_end.x_t");
                leg2_m45_20_end.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\bianbao\\x_t_hex_bend\\leg2_-45_20_end.x_t");
                leg2_45_20.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\bianbao\\x_t_hex_bend\\leg2_45_20.x_t");

                // leg3

                leg3_z.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\bianbao\\x_t_hex_bend\\leg3_z.x_t");
                leg3_75.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\bianbao\\x_t_hex_bend\\leg3_75.x_t");
                leg3_m45_20.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\bianbao\\x_t_hex_bend\\leg3_-45_20.x_t");
                leg3_45_20_end.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\bianbao\\x_t_hex_bend\\leg3_45_20_end.x_t");
                leg3_75_end.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\bianbao\\x_t_hex_bend\\leg3_75_end.x_t");
                leg3_m45_20_end.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\bianbao\\x_t_hex_bend\\leg3_-45_20_end.x_t");
                leg3_45_20.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\bianbao\\x_t_hex_bend\\leg3_45_20.x_t");

                // leg4

                leg4_z.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\bianbao\\x_t_hex_bend\\leg4_z.x_t");
                leg4_75.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\bianbao\\x_t_hex_bend\\leg4_75.x_t");
                leg4_m45_20.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\bianbao\\x_t_hex_bend\\leg4_-45_20.x_t");
                leg4_45_20_end.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\bianbao\\x_t_hex_bend\\leg4_45_20_end.x_t");
                leg4_75_end.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\bianbao\\x_t_hex_bend\\leg4_75_end.x_t");
                leg4_m45_20_end.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\bianbao\\x_t_hex_bend\\leg4_-45_20_end.x_t");
                leg4_45_20.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\bianbao\\x_t_hex_bend\\leg4_45_20.x_t");

                // leg5

                leg5_z.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\bianbao\\x_t_hex_bend\\leg5_z.x_t");
                leg5_75.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\bianbao\\x_t_hex_bend\\leg5_75.x_t");
                leg5_m45_20.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\bianbao\\x_t_hex_bend\\leg5_-45_20.x_t");
                leg5_45_20_end.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\bianbao\\x_t_hex_bend\\leg5_45_20_end.x_t");
                leg5_75_end.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\bianbao\\x_t_hex_bend\\leg5_75_end.x_t");
                leg5_m45_20_end.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\bianbao\\x_t_hex_bend\\leg5_-45_20_end.x_t");
                leg5_45_20.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\bianbao\\x_t_hex_bend\\leg5_45_20.x_t");

                // leg6

                leg6_z.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\bianbao\\x_t_hex_bend\\leg6_z.x_t");
                leg6_75.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\bianbao\\x_t_hex_bend\\leg6_75.x_t");
                leg6_m45_20.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\bianbao\\x_t_hex_bend\\leg6_-45_20.x_t");
                leg6_45_20_end.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\bianbao\\x_t_hex_bend\\leg6_45_20_end.x_t");
                leg6_75_end.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\bianbao\\x_t_hex_bend\\leg6_75_end.x_t");
                leg6_m45_20_end.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\bianbao\\x_t_hex_bend\\leg6_-45_20_end.x_t");
                leg6_45_20.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\bianbao\\x_t_hex_bend\\leg6_45_20.x_t");

                //add joints//

                // leg1
                auto& leg1_r1 = hex->addRevoluteJoint(leg1_75, leg1_z, leg1_pe[0], std::array<double, 3>{1, 0, 0}.data()); //虎克铰 x轴
                auto& leg1_r2 = hex->addRevoluteJoint(leg1_z, body, leg1_pe[1], std::array<double, 3>{0, 0, 1}.data()); //虎克铰 z轴
                auto& leg1_r3 = hex->addRevoluteJoint(leg1_m45_20, leg1_75, leg1_pe[2], std::array<double, 3>{0.7071,    0.3577, - 0.6100}.data()); //j1
                auto& leg1_r4 = hex->addRevoluteJoint(leg1_45_20_end, leg1_m45_20, leg1_pe[3], std::array<double, 3>{1, 0, 0}.data()); //j2
                auto& leg1_r5 = hex->addRevoluteJoint(leg1_75_end, leg1_45_20_end, leg1_pe[4], std::array<double, 3>{0.7071, - 0.3494, - 0.6147}.data());//j3
                auto& leg1_r6 = hex->addRevoluteJoint(leg1_m45_20_end, leg1_75_end, leg1_pe[5], std::array<double, 3>{0.7071, - 0.5034,    0.4966}.data());//j4
                auto& leg1_r7 = hex->addRevoluteJoint(leg1_45_20, leg1_m45_20_end, leg1_pe[6], std::array<double, 3>{1, 0, 0}.data());//j5
                auto& leg1_r8 = hex->addRevoluteJoint(leg1_75, leg1_45_20, leg1_pe[7], std::array<double, 3>{0.7071,    0.4966,    0.5034}.data());//j6
                


                // leg2
                auto& leg2_r1 = hex->addRevoluteJoint(leg2_75, leg2_z, leg2_pe[0], std::array<double, 3>{0.5, 0, 0.8660}.data()); 
                auto& leg2_r2 = hex->addRevoluteJoint(leg2_z, body, leg2_pe[1], std::array<double, 3>{-0.8660, 0, 0.5}.data()); 
                auto& leg2_r3 = hex->addRevoluteJoint(leg2_m45_20, leg2_75, leg2_pe[2], std::array<double, 3>{0.8818,    0.3577,    0.3074}.data());
                auto& leg2_r4 = hex->addRevoluteJoint(leg2_45_20_end, leg2_m45_20, leg2_pe[3], std::array<double, 3>{0.5, 0, 0.8660}.data());
                auto& leg2_r5 = hex->addRevoluteJoint(leg2_75_end, leg2_45_20_end, leg2_pe[4], std::array<double, 3>{0.8859, - 0.3494,    0.3050}.data());
                auto& leg2_r6 = hex->addRevoluteJoint(leg2_m45_20_end, leg2_75_end, leg2_pe[5], std::array<double, 3>{-0.0765, - 0.5034,    0.8607}.data());
                auto& leg2_r7 = hex->addRevoluteJoint(leg2_45_20, leg2_m45_20_end, leg2_pe[6], std::array<double, 3>{0.5, 0, 0.8660}.data());
                auto& leg2_r8 = hex->addRevoluteJoint(leg2_75, leg2_45_20, leg2_pe[7], std::array<double, 3>{-0.0824,    0.4966,    0.8641}.data());

                // leg3
                auto& leg3_r1 = hex->addRevoluteJoint(leg3_75, leg3_z, leg3_pe[0], std::array<double, 3>{-0.5000,         0,    0.8660}.data());
                auto& leg3_r2 = hex->addRevoluteJoint(leg3_z, body, leg3_pe[1], std::array<double, 3>{-0.8660,         0, - 0.5000}.data());
                auto& leg3_r3 = hex->addRevoluteJoint(leg3_m45_20, leg3_75, leg3_pe[2], std::array<double, 3>{0.1747,    0.3577,    0.9174}.data());
                auto& leg3_r4 = hex->addRevoluteJoint(leg3_45_20_end, leg3_m45_20, leg3_pe[3], std::array<double, 3>{ -0.5000,         0,    0.8660}.data());
                auto& leg3_r5 = hex->addRevoluteJoint(leg3_75_end, leg3_45_20_end, leg3_pe[4], std::array<double, 3>{ 0.1788, - 0.3494,    0.9197}.data());
                auto& leg3_r6 = hex->addRevoluteJoint(leg3_m45_20_end, leg3_75_end, leg3_pe[5], std::array<double, 3>{-0.7836, - 0.5034,    0.3641}.data());
                auto& leg3_r7 = hex->addRevoluteJoint(leg3_45_20, leg3_m45_20_end, leg3_pe[6], std::array<double, 3>{-0.5000,         0,    0.8660}.data());
                auto& leg3_r8 = hex->addRevoluteJoint(leg3_75, leg3_45_20, leg3_pe[7], std::array<double, 3>{-0.7895,    0.4966,    0.3607}.data());

                // leg4
                auto& leg4_r1 = hex->addRevoluteJoint(leg4_75, leg4_z, leg4_pe[0], std::array<double, 3>{-1.0000,         0,    0.0000}.data());
                auto& leg4_r2 = hex->addRevoluteJoint(leg4_z, body, leg4_pe[1], std::array<double, 3>{-0.0000,         0, - 1.0000}.data());
                auto& leg4_r3 = hex->addRevoluteJoint(leg4_m45_20, leg4_75, leg4_pe[2], std::array<double, 3>{ -0.7071,    0.3577,    0.6100}.data());
                auto& leg4_r4 = hex->addRevoluteJoint(leg4_45_20_end, leg4_m45_20, leg4_pe[3], std::array<double, 3>{ -1.0000,         0,    0.0000}.data());
                auto& leg4_r5 = hex->addRevoluteJoint(leg4_75_end, leg4_45_20_end, leg4_pe[4], std::array<double, 3>{ -0.7071, - 0.3494,    0.6147}.data());
                auto& leg4_r6 = hex->addRevoluteJoint(leg4_m45_20_end, leg4_75_end, leg4_pe[5], std::array<double, 3>{-0.7071, - 0.5034, - 0.4966}.data());
                auto& leg4_r7 = hex->addRevoluteJoint(leg4_45_20, leg4_m45_20_end, leg4_pe[6], std::array<double, 3>{   -1.0000,         0,    0.0000}.data());
                auto& leg4_r8 = hex->addRevoluteJoint(leg4_75, leg4_45_20, leg4_pe[7], std::array<double, 3>{   -0.7071,    0.4966, - 0.5034}.data());

                // leg5
                auto& leg5_r1 = hex->addRevoluteJoint(leg5_75, leg5_z, leg5_pe[0], std::array<double, 3>{-0.5000,         0, - 0.8660}.data());
                auto& leg5_r2 = hex->addRevoluteJoint(leg5_z, body, leg5_pe[1], std::array<double, 3>{ 0.8660,         0, - 0.5000}.data());
                auto& leg5_r3 = hex->addRevoluteJoint(leg5_m45_20, leg5_75, leg5_pe[2], std::array<double, 3>{ -0.8818,    0.3577, - 0.3074}.data());
                auto& leg5_r4 = hex->addRevoluteJoint(leg5_45_20_end, leg5_m45_20, leg5_pe[3], std::array<double, 3>{-0.5000,         0, - 0.8660}.data());
                auto& leg5_r5 = hex->addRevoluteJoint(leg5_75_end, leg5_45_20_end, leg5_pe[4], std::array<double, 3>{-0.8859, - 0.3494, - 0.3050}.data());
                auto& leg5_r6 = hex->addRevoluteJoint(leg5_m45_20_end, leg5_75_end, leg5_pe[5], std::array<double, 3>{0.0765, - 0.5034, - 0.8607}.data());
                auto& leg5_r7 = hex->addRevoluteJoint(leg5_45_20, leg5_m45_20_end, leg5_pe[6], std::array<double, 3>{-0.5000,         0, - 0.8660}.data());
                auto& leg5_r8 = hex->addRevoluteJoint(leg5_75, leg5_45_20, leg5_pe[7], std::array<double, 3>{0.0824,    0.4966, - 0.8641}.data());

                // leg6
                auto& leg6_r1 = hex->addRevoluteJoint(leg6_75, leg6_z, leg6_pe[0], std::array<double, 3>{0.5000,         0, - 0.8660}.data());
                auto& leg6_r2 = hex->addRevoluteJoint(leg6_z, body, leg6_pe[1], std::array<double, 3>{0.8660,         0,    0.5000}.data());
                auto& leg6_r3 = hex->addRevoluteJoint(leg6_m45_20, leg6_75, leg6_pe[2], std::array<double, 3>{-0.1747,    0.3577, - 0.9174}.data());
                auto& leg6_r4 = hex->addRevoluteJoint(leg6_45_20_end, leg6_m45_20, leg6_pe[3], std::array<double, 3>{0.5000,         0, - 0.8660}.data());
                auto& leg6_r5 = hex->addRevoluteJoint(leg6_75_end, leg6_45_20_end, leg6_pe[4], std::array<double, 3>{-0.1788, - 0.3494, - 0.9197}.data());
                auto& leg6_r6 = hex->addRevoluteJoint(leg6_m45_20_end, leg6_75_end, leg6_pe[5], std::array<double, 3>{ 0.7836, - 0.5034, - 0.3641}.data());
                auto& leg6_r7 = hex->addRevoluteJoint(leg6_45_20, leg6_m45_20_end, leg6_pe[6], std::array<double, 3>{0.5000,         0, - 0.8660}.data());
                auto& leg6_r8 = hex->addRevoluteJoint(leg6_75, leg6_45_20, leg6_pe[7], std::array<double, 3>{0.7895,    0.4966, - 0.3607}.data());

                //add motion//

                //leg1//
                auto& leg1_m1 = hex->addMotion(leg1_r1); //虎克铰X轴
                auto& leg1_m2 = hex->addMotion(leg1_r2); //虎克铰Y轴
                auto& leg1_m3 = hex->addMotion(leg1_r4); //J2

                //leg2//
                auto& leg2_m1 = hex->addMotion(leg2_r1); 
                auto& leg2_m2 = hex->addMotion(leg2_r2); 
                auto& leg2_m3 = hex->addMotion(leg2_r4); 

                //leg3//
                auto& leg3_m1 = hex->addMotion(leg3_r1); 
                auto& leg3_m2 = hex->addMotion(leg3_r2); 
                auto& leg3_m3 = hex->addMotion(leg3_r4); 

                //leg4//
                auto& leg4_m1 = hex->addMotion(leg4_r1); 
                auto& leg4_m2 = hex->addMotion(leg4_r2); 
                auto& leg4_m3 = hex->addMotion(leg4_r4); 

                //leg5//
                auto& leg5_m1 = hex->addMotion(leg5_r1); 
                auto& leg5_m2 = hex->addMotion(leg5_r2); 
                auto& leg5_m3 = hex->addMotion(leg5_r4); 

                //leg6//
                auto& leg6_m1 = hex->addMotion(leg6_r1); 
                auto& leg6_m2 = hex->addMotion(leg6_r2); 
                auto& leg6_m3 = hex->addMotion(leg6_r4); 

                //add end-effector//
                auto body_ee_maki = body.addMarker("body_ee_mak_i");
                auto body_ee_makj = hex->ground().addMarker("body_ee_mak_j");

                auto& body_ee = hex->generalMotionPool().add<aris::dynamic::GeneralMotion>("body_ee", &body_ee_maki, &body_ee_makj);
                auto& leg1_ee = hex->addPointMotion(leg1_75_end, hex->ground(), ee_pos[0]);
                hex->ground().markerPool().back().setPrtPe(std::array<double, 6>{0, 0, 0, 0, 0, 0}.data());  //这个地方是height吗
                auto& leg2_ee = hex->addPointMotion(leg2_75_end, hex->ground(), ee_pos[1]);
                hex->ground().markerPool().back().setPrtPe(std::array<double, 6>{0, 0, 0, 0, 0, 0}.data());
                auto& leg3_ee = hex->addPointMotion(leg3_75_end, hex->ground(), ee_pos[2]);
                hex->ground().markerPool().back().setPrtPe(std::array<double, 6>{0, 0, 0, 0, 0, 0}.data());
                auto& leg4_ee = hex->addPointMotion(leg4_75_end, hex->ground(), ee_pos[3]);
                hex->ground().markerPool().back().setPrtPe(std::array<double, 6>{0, 0, 0, 0, 0, 0}.data());
                auto& leg5_ee = hex->addPointMotion(leg5_75_end, hex->ground(), ee_pos[4]);
                hex->ground().markerPool().back().setPrtPe(std::array<double, 6>{0, 0, 0, 0, 0, 0}.data());
                auto& leg6_ee = hex->addPointMotion(leg6_75_end, hex->ground(), ee_pos[5]);
                hex->ground().markerPool().back().setPrtPe(std::array<double, 6>{0, 0, 0, 0, 0, 0}.data());



                auto& inverse_kinematic_solver = hex->solverPool().add<aris::dynamic::InverseKinematicSolver>();

                inverse_kinematic_solver.setMaxError(1e-7);
                inverse_kinematic_solver.setIterCount(1000);


                auto& forward_kinematic_solver = hex->solverPool().add<aris::dynamic::ForwardKinematicSolver>();
                auto& inverse_dynamic_solver = hex->solverPool().add<aris::dynamic::InverseDynamicSolver>();
                auto& forward_dynamic_solver = hex->solverPool().add<aris::dynamic::ForwardDynamicSolver>();

                auto& stand_universal = hex->solverPool().add<aris::dynamic::UniversalSolver>();

                //添加仿真器和仿真结果//
                auto& adams = hex->simulatorPool().add<aris::dynamic::AdamsSimulator>();
                auto& result = hex->simResultPool().add<aris::dynamic::SimResult>();

                hex->init();

                // 设置默认拓扑结构 //
                for (auto& m : hex->motionPool())m.activate(true);
                for (auto& gm : hex->generalMotionPool())gm.activate(false);

                return hex;

            }






auto createControllerHexapod()->std::unique_ptr<aris::control::Controller>
{
    std::unique_ptr<aris::control::Controller> controller(new aris::control::EthercatController);

    for (aris::Size i = 0; i < 1; ++i)
    {
#ifdef ARIS_USE_ETHERCAT_SIMULATION
        double pos_offset[18]
        {
            0,0,0,
            0,0,0,
            0,0,0,
            0,0,0,
            0,0,0,
            0,0,0
        };
#else
        double pos_offset[18]
        {
            0,0,0,
            0,0,0,
            0,0,0,
            0,0,0,
            0,0,0,
            0,0,0

        };
#endif
        double pos_factor[18] //偏置系数
        {
            2000/PI,2000/PI,2000/PI,
            2000/PI,2000/PI,2000/PI,
            2000/PI,2000/PI,2000/PI,
            2000/PI,2000/PI,2000/PI,
            2000/PI,2000/PI,2000/PI,
            2000/PI,2000/PI,2000/PI
        };
        double max_pos[18] //最大位置
        {
            500*PI,500*PI,500*PI,
            500*PI,500*PI,500*PI,
            500*PI,500*PI,500*PI,
            500*PI,500*PI,500*PI,
            500*PI,500*PI,500*PI,
            500*PI,500*PI,500*PI
        };
        double min_pos[18] //最小位置
        {
            -500*PI,-500*PI,-500*PI,
            -500*PI,-500*PI,-500*PI,
            -500*PI,-500*PI,-500*PI,
            -500*PI,-500*PI,-500*PI,
            -500*PI,-500*PI,-500*PI,
            -500*PI,-500*PI,-500*PI
        };
        double max_vel[18]  //最大速度
        {
            100 * PI, 100 * PI,  100 * PI,
            330 / 60 * 2 * PI, 330 / 60 * 2 * PI,  330 / 60 * 2 * PI,
            330 / 60 * 2 * PI, 330 / 60 * 2 * PI,  330 / 60 * 2 * PI,
            330 / 60 * 2 * PI, 330 / 60 * 2 * PI,  330 / 60 * 2 * PI,
            330 / 60 * 2 * PI, 330 / 60 * 2 * PI,  330 / 60 * 2 * PI,
            330 / 60 * 2 * PI, 330 / 60 * 2 * PI,  330 / 60 * 2 * PI
        };
        double max_acc[18]  //最大加速度
        {
            3000,  3000,  3000,
            3000,  3000,  3000,
            3000,  3000,  3000,
            3000,  3000,  3000,
            3000,  3000,  3000,
            3000,  3000,  3000
        };

        int phy_id[18]={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17};


        std::string xml_str =
            "<EthercatMotor phy_id=\"" + std::to_string(phy_id[i]) + "\" product_code=\"0x00\""
            " vendor_id=\"0x00\" revision_num=\"0x00\" dc_assign_activate=\"0x0300\""
            " min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]) + "\" min_vel=\"" + std::to_string(-max_vel[i]) + "\""
            " max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"10.0\" max_vel_following_error=\"20.0\""
            " home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor[i]) + "\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\">"
            "	<SyncManagerPoolObject>"
            "		<SyncManager is_tx=\"false\"/>"
            "		<SyncManager is_tx=\"true\"/>"
            "		<SyncManager is_tx=\"false\">"
            "			<Pdo index=\"0x1600\" is_tx=\"false\">"
            "				<PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
            "				<PdoEntry name=\"target_vel\" index=\"0x60FF\" subindex=\"0x00\" size=\"32\"/>"
//            "				<PdoEntry name=\"targer_toq\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
//            "				<PdoEntry name=\"max_toq\" index=\"0x6072\" subindex=\"0x00\" size=\"16\"/>"
            "				<PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
            "				<PdoEntry name=\"mode_of_operation\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
            "			</Pdo>"
            "		</SyncManager>"
            "		<SyncManager is_tx=\"true\">"
            "			<Pdo index=\"0x1A00\" is_tx=\"true\">"
            "				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
            "				<PdoEntry name=\"mode_of_display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
            "				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
            "				<PdoEntry name=\"vel_actual_value\" index=\"0x606c\" subindex=\"0x00\" size=\"32\"/>"
//            "				<PdoEntry name=\"toq_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
            "				<PdoEntry name=\"digital_inputs\" index=\"0x60FD\" subindex=\"0x00\" size=\"32\"/>"
            "			</Pdo>"
            "		</SyncManager>"
            "	</SyncManagerPoolObject>"
            "</EthercatMotor>";


        auto &s = controller->slavePool().add<aris::control::EthercatMotor>();
        aris::core::fromXmlString(s,xml_str);

#ifdef WIN32
        dynamic_cast<aris::control::EthercatMotor&>(controller->slavePool().back()).setVirtual(true);
#endif

#ifndef WIN32
        dynamic_cast<aris::control::EthercatMotor&>(controller->slavePool().back()).scanInfoForCurrentSlave();
#endif

        dynamic_cast<aris::control::EthercatMotor&>(controller->slavePool().back()).setDcAssignActivate(0x300);
        dynamic_cast<aris::control::EthercatMotor&>(controller->slavePool().back()).setControlWord(0x00);
        dynamic_cast<aris::control::EthercatMotor&>(controller->slavePool().back()).setModeOfOperation(0x08);
        dynamic_cast<aris::control::EthercatMotor&>(controller->slavePool().back()).setTargetPos(0.0);
    };
    return controller;
}
auto createPlanHexapod()->std::unique_ptr<aris::plan::PlanRoot>
{
    std::unique_ptr<aris::plan::PlanRoot> plan_root(new aris::plan::PlanRoot);

    plan_root->planPool().add<aris::plan::Enable>();
    plan_root->planPool().add<aris::plan::Disable>();
    plan_root->planPool().add<aris::plan::Home>();
    plan_root->planPool().add<aris::plan::Mode>();
    plan_root->planPool().add<aris::plan::Show>();
    plan_root->planPool().add<aris::plan::Sleep>();
    plan_root->planPool().add<aris::plan::Clear>();
    plan_root->planPool().add<aris::plan::Recover>();
    auto &rs = plan_root->planPool().add<aris::plan::Reset>();
    rs.command().findParam("pos")->setDefaultValue("{0.5,0.392523364485981,0.789915966386555,0.5,0.5,0.5}");

    auto &mvaj = plan_root->planPool().add<aris::plan::MoveAbsJ>();
    mvaj.command().findParam("vel")->setDefaultValue("0.1");

    plan_root->planPool().add<aris::plan::MoveL>();
    plan_root->planPool().add<aris::plan::MoveJ>();
    plan_root->planPool().add<aris::plan::GetXml>();
    plan_root->planPool().add<aris::plan::SetXml>();
    plan_root->planPool().add<aris::plan::Start>();
    plan_root->planPool().add<aris::plan::Stop>();

    //驱动命令
    plan_root->planPool().add<ReadCurrentPos>();
    plan_root->planPool().add<TCurve2Test>();
    plan_root->planPool().add<Home>();
    plan_root->planPool().add<Home2>();
    plan_root->planPool().add<MoveJointAll>();
    plan_root->planPool().add<MoveJointAllCos>();
    plan_root->planPool().add<HexForward>();
    plan_root->planPool().add<HexLateral>();
    plan_root->planPool().add<HexTurn>();
    plan_root->planPool().add<HexTetrapod>();
    plan_root->planPool().add<Test>();

    //仿真命令
    plan_root->planPool().add<ReadCurrentPos>();
    plan_root->planPool().add<HexDynamicForwardTest>();
    plan_root->planPool().add<HexDynamicBackTest>();
    plan_root->planPool().add<HexDynamicRightTest>();
    plan_root->planPool().add<HexDynamicLeftTest>();
    plan_root->planPool().add<HexDynamicTurnRightTest>();
    plan_root->planPool().add<HexDynamicTurnLeftTest>();
    plan_root->planPool().add<HexDynamicTetrapodTest>();
    plan_root->planPool().add<SingleLeg>();
    plan_root->planPool().add<MoveBody>();




    return plan_root;
}

}
