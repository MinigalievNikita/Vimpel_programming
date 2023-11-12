#include <iostream>
#include <vector>
#include <list>
#include <math.h>
#include <map>
#include <unordered_map>
#include <limits>
#include <fstream>
#include <thread>

#define MAX_BMs  15;
#define maxBMs  5;
#define STEP_T  0.5;
#define MAX_DIST  5;
#define MIN_US  5;
#define MAX_US  89;
#define MAX_DELAY 2;
#define G 9.8
#define PI 3.1415


struct tPoint {
    double x, y;

    tPoint(double p_x = 0, double p_y = 0) : x(p_x), y(p_y){}

    static const tPoint& none() {
        static tPoint s { std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN() };
        return s;
    }
};

enum Level : int {
        TRACE = 0,
        DEBUG,
        INFO,
        WARN,
        ERROR,
};

class Singleton {
public:
    template <typename T> static const T& self() {
        static T v;
        return v;
    }
};

class LoggerWork {
public:
    void log(Level p_level, const std::string& p_message) const {
        if (p_level < c_level) return;
        if (logs_messages.is_open()){
        logs_messages << p_message << std::endl;
        }
    }

    void loglevel(Level p_level) const {
            c_level = p_level;    
    }

    Level loglevel() const {
        return c_level;
    }

    LoggerWork() {
        logs_messages.open("logsWork.txt");
    }

    ~LoggerWork(){
        logs_messages.close();
    }

 private:
    mutable std::ofstream logs_messages;
    mutable Level c_level;
};

class LoggerBMs {
public:
    void log(Level p_level, const std::string& p_message) const {
        if (p_level < c_level) return;
        if (logs_messages.is_open()){
        logs_messages << p_message << std::endl;
        }
    }

    void log(Level p_level, int id, tPoint position, double time, const std::string& p_message) const {
        if (p_level < c_level) return;
        if (logs_messages.is_open()){
        logs_messages << "Id = " << id << "Position: x = " << position.x<< "y = " << position.y << "Time is = " << time << p_message << std::endl;
        }
    }

    void loglevel(Level p_level) const {
            c_level = p_level;    
    }

    Level loglevel() const {
        return c_level;
    }

    LoggerBMs() {
        logs_messages.open("logsBMs.txt");
    }

    ~LoggerBMs(){
        logs_messages.close();
    }

 private:
    mutable std::ofstream logs_messages;
    mutable Level c_level;
};


//!
class Config {
public:
	static const Config& self(){
        static Config l;
        return l;
    };

    // Контрактные ограничения
	int premittedBMs() const{
        return MAX_BMs;
    }; //!< максимально допустимое количество БР, одновременно находящихся в полёте

    // Параметры для имитации
	int maxBM() const{
        return maxBMs;
    }; //!< количество БР в ударе
	double stepT() const{
        return STEP_T;
    }; //!< шаг модельного времени

    // Параметры для рандомизации тестовых данных
	double maxS() const{
        return MAX_DIST;
    };  //!< максимальное удаление точки старта от [0;0]
	double minUS() const{
        return MIN_US;
    }; //!< минимальный угол бросания
	double maxUS() const{
        return MAX_US;
    }; //!< максимальный угол бросания
	double maxT() const{
        return MAX_DELAY;
    };  //!< максимальная задержка времени старта
};

//! Абстрактный описатель элемента
class IBmElement {
    public:
		//! \return точку старта БР
		virtual const tPoint& launchPoint() const = 0;

		//! \return точку падения БР
		virtual const tPoint& boomPoint() const = 0;

		//! \return момент начала движения
		virtual double launchTime() const = 0;

		//! \return момент окончания движения
		virtual double boomTime() const = 0;

		//заполнение мапы <время - точка>
        virtual void positionAt(double p_time) const = 0;

        //возвращает координату елемента в дискрет p_time
        virtual const tPoint getMapPosition(double p_time) const = 0;

        virtual const double getMapTime(double p_time) const = 0;
};


//! Абстрактный описатель полётного задания
class IFlightTask {
    public:
		//! \return идентификатор задания
		virtual int id() const = 0;

		//! \return точку старта БР
		virtual const tPoint& launchPoint() const = 0;

		//! \return точку падения БР
		virtual const tPoint& boomPoint() const = 0;

		//! \return момент начала движения
		virtual double launchTime() const = 0;

		//! \return угол бросания. Фактически - тип траектории (настильная/навесная)
		virtual double angle() const = 0;

		//! \return тип БР (фактически - ключ к некоторому справочнику ТТХ БР)
		virtual int type() const = 0;

        virtual double tFlight() = 0;

        virtual tPoint VStart(double time) = 0;

        virtual tPoint getPosition(double t) const = 0;
};


//!
class tElement : public IBmElement {
public:
    tElement(IFlightTask* ff){
        task = ff;
    }
	const tPoint& launchPoint() const{
        return task->launchPoint();
    };

    const tPoint& boomPoint() const{
        return task->boomPoint();
    };

	double launchTime() const{
        return task->launchTime();
    };

    double boomTime() const{
        return task->tFlight();
    };

	//заполнение мапы <время - точка>
    void positionAt(double p_time) const{
        position[p_time] = task->getPosition(p_time);
    };

    const tPoint getMapPosition(double p_time) const{
        if((*position.upper_bound(p_time)).first != (*position.lower_bound(p_time)).first){
            return position[p_time];
        }
        return (*position.upper_bound(p_time - 1)).second;
    }

    const double getMapTime(double p_time) const{
        if((*position.upper_bound(p_time)).first != (*position.lower_bound(p_time)).first){
            return p_time;
        }
        return (*position.upper_bound(p_time - 1)).first;
    }

private:
    IFlightTask* task;
    tPoint x;
    mutable std::map<double, tPoint> position;
};


//! Класс, реализующий модель движения БР по параболе
class ParabolicModel : public IFlightTask {
public:
    ParabolicModel(unsigned long id_t, int p_type_t, double p_t0_t, double p_angle_t, const tPoint p_launch_t, const tPoint p_boom_t){
        id_ = id_t;
        p_type = p_type_t;
        p_t0 = p_t0_t;
        p_angle = p_angle_t;
        p_launch = p_launch_t;
        p_boom = p_boom_t;
    }  

    int id() const{
        return id_;
    }

    const tPoint& launchPoint() const{
        return p_launch;
    }

    const tPoint& boomPoint() const{
        return p_boom;
    }

    double launchTime() const{
        return p_t0;
    }

    double angle() const{
        return p_angle;
    }

    int type() const{
        return p_type;
    }
    double tFlight() {
        time_flight = (double)sqrt((2*(p_boom.x - p_launch.x) / G) * tan(p_angle * PI / 180));
        tPoint v = VStart(time_flight);
        return time_flight + p_t0;
    }

    tPoint VStart(double time) {
        V0.x = (p_boom.x - p_launch.x) / time;
        V0.y = time * G / 2;
        return V0;
    }

    tPoint getPosition(double t) const{
        t = t - p_t0;
        tPoint temp;
        temp.x = p_launch.x + V0.x * t;
        temp.y = V0.y * t - G * t * t /2;
        return temp;
    }
  

private:
    int id_, p_type;
    double p_t0, p_angle;
    tPoint p_launch, p_boom;
    double time_flight;
    tPoint V0;
};



class StrikeScenario {
public:
    typedef std::unordered_map<int, std::vector<int>> tStrike;

public:
    //! Добавляет полетное задание в сценарий
    IFlightTask* addFlightTask(int p_type, double p_t0, double p_angle, const tPoint& p_launch, const tPoint& p_boom) {
        IFlightTask* ft = nullptr;

        if (p_type == 1) {
            ft = new ParabolicModel(tasks.size() + 1, p_type, p_t0, p_angle, p_launch, p_boom);
        } else {
            Singleton::self<LoggerBMs>().log(ERROR,tasks.size() + 1, p_launch, -1,  " TYPE IS UNACCESSIBLE");
        }
        
        if (ft != nullptr) {
            tasks.insert( { ft->id(), ft } );
        }

        return ft;
    }

    //!
    int prepare(tStrike& p_strike) {
        for (const auto& ft : tasks) {
            IBmElement* e = elements.insert( { ft.second->id(), new tElement(ft.second) } ).first->second;
            
            if(e->launchPoint().x > e->boomPoint().x)
                Singleton::self<LoggerBMs>().log(ERROR, ft.second->id(), e->launchPoint(), -1, "launchPoint > boomPoint");
            
            std::vector<int> v;
            v.push_back(ft.second->id());
            p_strike[ft.second->id()] = v;
            
            double launchtime = e->launchTime();
            double boomtime = e->boomTime();
            if(minlaunchtime > launchtime)
                minlaunchtime = launchtime;
            if(maxboomtime < boomtime)
                maxboomtime = boomtime;
            BMlimits[launchtime] = 1;
            BMlimits[boomtime] = -1;
            for (double t = launchtime, tE = boomtime; t < tE; t += 1.0) {
                // заполнить таблицу (собственную или в IBmElement) с опорными значениями для интерполяции
                e->positionAt(t);
            }
            // проверить, что для момента e->boomTime() данные тоже посчитаны
            e->positionAt(boomtime);

            if(false){
                Singleton::self<LoggerWork>().log(ERROR, " WRONG TIME TABLE");
                return -1;
            }
        }
        //проверка контрактных ограничений
        int sum = 0;
        for (const auto& it : BMlimits){
            sum += it.second;
            if(sum > Config::self().premittedBMs())
                Singleton::self<LoggerWork>().log(ERROR, " TOO MUCH BMs");
        }

        return 0;
    } 

    // int prepare(tStrike& p_strike) {
    //     std::vector<std::thread> vecOfThreads;
    //     //vecOfThreads.push_back(std::thread(preparetask));
    //     for (const auto& ft : tasks) {
    //         std::thread th1(preparetask, ft, p_strike);
    //         std::thread th2(preparetask, ++ft, p_strike);
    //         std::thread th3(preparetask, ++ft, p_strike);
    //         std::thread th4(preparetask, ++ft, p_strike);
    //     }
    //      //проверка контрактных ограничений
    //     int sum = 0;
    //     for (const auto& it : BMlimits){
    //          sum += it.second;
    //          if(sum > Config::self().premittedBMs())
    //              Singleton::self<LoggerWork>().log(ERROR, " TOO MUCH BMs");
    //     }

    // return 0;
    // }

    // void preparetask(const std::pair<const int, IFlightTask *> &ft, tStrike& p_strike){
    //     IBmElement* e = elements.insert( { ft.second->id(), new tElement(ft.second) } ).first->second;
            
    //         if(e->launchPoint().x > e->boomPoint().x)
    //             Singleton::self<LoggerBMs>().log(ERROR, ft.second->id(), e->launchPoint(), -1, "launchPoint > boomPoint");
            
    //         std::vector<int> v;
    //         v.push_back(ft.second->id());
    //         p_strike[ft.second->id()] = v;
            
    //         double launchtime = e->launchTime();
    //         double boomtime = e->boomTime();
    //         if(minlaunchtime > launchtime)
    //             minlaunchtime = launchtime;
    //         if(maxboomtime < boomtime)
    //             maxboomtime = boomtime;
    //         BMlimits[launchtime] = 1;
    //         BMlimits[boomtime] = -1;
    //         for (double t = launchtime, tE = boomtime; t < tE; t += 1.0) {
    //             // заполнить таблицу (собственную или в IBmElement) с опорными значениями для интерполяции
    //             e->positionAt(t);
    //         }
    //         // проверить, что для момента e->boomTime() данные тоже посчитаны
    //         e->positionAt(boomtime);

    //         if(false){
    //             Singleton::self<LoggerWork>().log(ERROR, " WRONG TIME TABLE");
    //         }
    // }

    //!
    tPoint positionOf(int p_id, double p_time) {
        if(elements[p_id]->launchTime() >= p_time)
            return elements[p_id]->launchPoint();
        if(elements[p_id]->boomTime() <= p_time)
            return elements[p_id]->boomPoint();
        tPoint x0 = elements[p_id]->getMapPosition(p_time); //return map(p_time) or map от меньше p_time
        tPoint x1 = elements[p_id]->getMapPosition(p_time + 1);
        x0.x = x0.x + (x1.x - x0.x) * (p_time - elements[p_id]->getMapTime(p_time));
        x0.y = x0.y + (x1.y - x0.y) * (p_time - elements[p_id]->getMapTime(p_time));
        return x0;
    }

    double minLaunchTime() {
        return minlaunchtime;
    }
    double maxBoomTime() {
        return maxboomtime;
    }

    int waitForAll(){
        return 0;
    }

private:
    double minlaunchtime = 10000, maxboomtime = 0; 
    std::map<double, int> BMlimits;
    std::unordered_map<int, IFlightTask*> tasks;
    std::unordered_map<int, IBmElement*> elements;
};


//!
class MathHelper {
public:
    static const double g;
    static const double eps;

public:
    static double random(double p_min, double p_max) {
        return p_min + (p_max - p_min)*rand() / static_cast<double>(RAND_MAX);
    }
};



int main() {
    Singleton::self<LoggerWork>().loglevel(TRACE);
    Singleton::self<LoggerBMs>().loglevel(TRACE);
    Singleton::self<LoggerWork>().log(INFO, " START");
    StrikeScenario ss;
    const Config& cfg = Config::self();
    for (int i = 0; i < cfg.maxBM(); ++i) {
        (void)ss.addFlightTask(1,                                               //!< тип БР пока всегда 1
                               MathHelper::random(0, cfg.maxT()),               //!< задержка старта
                               MathHelper::random(cfg.minUS(), cfg.maxUS()),    //!< угол бросания
                               tPoint(MathHelper::random(0, cfg.maxS())),       //!< точка старта
                               tPoint(cfg.maxS() + MathHelper::random(0, cfg.maxS()))    //!< точка падения
                               );
        
    }
    Singleton::self<LoggerWork>().log(INFO, " CONFIG DONE");
    std::unordered_map<int, std::vector<int>> elements; // id (id полетного задания, который запомнили) -> массив движущихся элементов  
    int ok = ss.prepare(elements);
    if (ok != 0) {
        Singleton::self<LoggerWork>().log(ERROR, "FatalErrorInPrepare");
        return -1;
    } else {
        for (double t = ss.minLaunchTime(); t < ss.maxBoomTime() + cfg.stepT(); t += cfg.stepT()) {
            for (const auto& e : elements) {
                //for (int id : e.second) {
                    
                    auto p = ss.positionOf(e.second[0], t);
                    Singleton::self<LoggerBMs>().log(INFO, e.second[0], p, t, " TIMESTEP DONE OK");
                //}
            }
            Singleton::self<LoggerWork>().log(INFO, " OK");
        }
    }
    Singleton::self<LoggerWork>().log(INFO, " END OF PROGRAM");
}
