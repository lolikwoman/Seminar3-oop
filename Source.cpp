#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <clocale>

using namespace std;

// Задача 1. Класс Sensor
class Sensor {
private:
    double signalStrength;
    bool isActive;
public:
    Sensor(double strength) : signalStrength(strength), isActive(true) {}

    void activate() { isActive = true; }
    void deactivate() { isActive = false; }

    void setSignalStrength(double s) {
        signalStrength = (s < 0) ? 0 : s;
    }

    double getSignalStrength() { return signalStrength; }

    void printStatus() {
        cout << "Датчик " << (isActive ? "активен" : "выключен")
            << " | Сила сигнала: " << signalStrength << " дБ" << endl;
    }
};

// Задача 2. Класс Trajectory
class Trajectory {
private:
    double startX, startY;
    double angle;
public:
    Trajectory() : startX(0), startY(0), angle(0) {}
    Trajectory(double x, double y, double ang) : startX(x), startY(y), angle(ang) {}

    void printTrajectory() {
        cout << "Старт: (" << startX << ", " << startY
            << "), угол: " << angle << "°" << endl;
    }
};

// Задача 3. Класс Engine
class Engine {
private:
    double thrust;
    double fuelFlow;
public:
    Engine(double T, double F) : thrust(T), fuelFlow(F) {}

    double getSpecificImpulse() {
        if (fuelFlow == 0) return 0;
        return thrust / (fuelFlow * 9.81);
    }

    void printInfo() {
        cout << fixed << setprecision(2);
        cout << "Тяга: " << thrust << " Н | Расход: " << fuelFlow
            << " кг/с | Удельный импульс: " << getSpecificImpulse() << " с" << endl;
    }
};

// Задача 4. Класс Gyroscope
class Gyroscope {
private:
    double angularVelocity;
    bool calibrationRequired;
public:
    Gyroscope(double vel, bool calib) : angularVelocity(vel), calibrationRequired(calib) {}

    void calibrate() {
        calibrationRequired = false;
    }

    void printStatus() {
        cout << "Скорость: " << angularVelocity << " °/с | Калибровка "
            << (calibrationRequired ? "требуется" : "не требуется") << endl;
    }
};

// Задача 5. Класс Autopilot
class Autopilot {
private:
    double courseAngle;
    double altitude;
public:
    Autopilot(double course, double alt) : courseAngle(course), altitude(alt) {}

    void changeCourse(double delta) {
        courseAngle += delta;
    }

    void changeAltitude(double delta) {
        altitude += delta;
    }

    void printStatus() {
        cout << "Курс: " << courseAngle << "°, высота: " << altitude << " м" << endl;
    }
};

// Задача 6. Класс RocketStage
class RocketStage {
private:
    double thrust;
    double burnTime;
    double mass;
public:
    RocketStage(double t, double bt, double m) : thrust(t), burnTime(bt), mass(m) {}

    double getDeltaV() {
        return (thrust * burnTime) / mass;
    }

    void printInfo() {
        cout << "Тяга: " << thrust << " Н, Время работы: " << burnTime
            << " с, Масса: " << mass << " кг" << endl;
    }
};

// Задача 7. Класс FlightComputer
class FlightComputer {
private:
    double altitude;
    double velocity;
    double fuel;
    double thrust;
public:
    FlightComputer(double alt, double vel, double f, double thr)
        : altitude(alt), velocity(vel), fuel(f), thrust(thr) {
    }

    void simulateStep(double dt) {
        if (fuel > 0) {
            velocity += (thrust / 1000 - 9.8) * dt;
            altitude += velocity * dt;
            fuel -= dt * 2;
            if (fuel < 0) fuel = 0;
        }
    }

    void printStatus() {
        cout << fixed << setprecision(1);
        cout << "Высота: " << altitude << " м, Скорость: " << velocity
            << " м/с, Топливо: " << fuel << " кг" << endl;
    }
};

// Задача 8. Класс NavigationSystem
class NavigationSystem {
private:
    double x, y;
    double velocityX, velocityY;
    bool gpsAvailable;
public:
    NavigationSystem(double posX, double posY, double velX, double velY, bool gps)
        : x(posX), y(posY), velocityX(velX), velocityY(velY), gpsAvailable(gps) {
    }

    void integratePosition(double dt) {
        x += velocityX * dt;
        y += velocityY * dt;
    }

    void correctGPS(double realX, double realY) {
        if (gpsAvailable) {
            x = (x + realX) / 2;
            y = (y + realY) / 2;
        }
    }

    void printPosition() {
        cout << "Скорректированные координаты: (" << x << ", " << y << ")" << endl;
    }
};

// Задача 9. Класс AutonomousControl
class AutonomousControl {
private:
    double altitude;
    double thrust;
    double targetAltitude;
public:
    AutonomousControl(double alt, double thr, double target)
        : altitude(alt), thrust(thr), targetAltitude(target) {
    }

    void updateControl() {
        if (altitude < targetAltitude) {
            thrust += 100; // увеличение тяги
        }
        else {
            thrust -= 100; // уменьшение тяги
        }
        if (thrust < 0) thrust = 0;
    }

    void simulateStep(double dt) {
        altitude += (thrust / 500) * dt;
    }

    void printStatus() {
        cout << "Высота: " << altitude << "м, тяга: " << thrust << endl;
    }
};

// Задача 10. Класс DroneFlight
class DroneFlight {
private:
    vector<double> x;
    vector<double> y;
    double totalDistance;
public:
    DroneFlight() : totalDistance(0) {
        x.push_back(0);
        y.push_back(0);
    }

    void addPoint(double newX, double newY) {
        double lastX = x.back();
        double lastY = y.back();
        double distance = sqrt(pow(newX - lastX, 2) + pow(newY - lastY, 2));
        totalDistance += distance;

        x.push_back(newX);
        y.push_back(newY);
    }

    double getTotalDistance() {
        return totalDistance;
    }

    void printPath() {
        cout << "Точки маршрута:" << endl;
        for (size_t i = 0; i < x.size(); i++) {
            cout << "(" << x[i] << ", " << y[i] << ")" << endl;
        }
    }
};

// Задача 11. Классы для комплексной системы
class Engine11 {
private:
    double thrust;
    double fuelFlow;
    double fuel;
public:
    Engine11(double thr, double flow, double f) : thrust(thr), fuelFlow(flow), fuel(f) {}

    double getThrust() { return thrust; }

    bool hasFuel() { return fuel > 0; }

    void burn(double dt) {
        if (fuel > 0) {
            double consumed = fuelFlow * dt;
            if (consumed > fuel) consumed = fuel;
            fuel -= consumed;
        }
    }

    double getFuel() { return fuel; }
};

class Navigation11 {
private:
    double altitude;
    double velocity;
    double acceleration;
    double mass;
public:
    Navigation11(double alt, double vel, double m) : altitude(alt), velocity(vel), mass(m), acceleration(0) {}

    void update(double thrust, double dt) {
        acceleration = thrust / mass - 9.81;
        velocity += acceleration * dt;
        altitude += velocity * dt;
    }

    void printStatus(double time) {
        cout << fixed << setprecision(2);
        cout << "t=" << time << "c | h=" << altitude << "м | v="
            << velocity << "м/с | a=" << acceleration << "м/с²" << endl;
    }

    double getAltitude() { return altitude; }
};

class AutonomousFlightSystem {
private:
    Engine11 engine;
    Navigation11 nav;
    double time;
public:
    AutonomousFlightSystem(Engine11 e, Navigation11 n) : engine(e), nav(n), time(0) {}

    void simulate(double dt, double totalTime) {
        while (time < totalTime && engine.hasFuel()) {
            engine.burn(dt);
            nav.update(engine.getThrust(), dt);
            nav.printStatus(time);
            time += dt;
        }
    }

    void printSummary() {
        cout << "--- Полет завершен ---" << endl;
        cout << "Оставшееся топливо: " << engine.getFuel() << " кг" << endl;
        cout << "Итоговая высота: " << nav.getAltitude() << " м" << endl;
    }
};


int main() {
    setlocale(LC_ALL, "Russian");

    int choice;

    do {
        cout << "\n=== МЕНЮ ВЫБОРА ЗАДАЧ ===" << endl;
        cout << "1. Датчик (Sensor)" << endl;
        cout << "2. Траектория (Trajectory)" << endl;
        cout << "3. Двигатель (Engine)" << endl;
        cout << "4. Гироскоп (Gyroscope)" << endl;
        cout << "5. Автопилот (Autopilot)" << endl;
        cout << "6. Многоступенчатая ракета" << endl;
        cout << "7. Управление полетом (FlightComputer)" << endl;
        cout << "8. Коррекция координат" << endl;
        cout << "9. Автономное управление высотой" << endl;
        cout << "10. Анализ маршрута дрона" << endl;
        cout << "11. Комплексная система моделирования" << endl;
        cout << "0. Выход" << endl;
        cout << "Выберите задачу: ";
        cin >> choice;

        switch (choice) {
        case 1: {
            cout << "\n=== Задача 1. Датчик ===" << endl;
            Sensor s1(45.5);
            s1.printStatus();
            s1.setSignalStrength(-10);
            s1.printStatus();
            s1.deactivate();
            s1.printStatus();
            break;
        }

        case 2: {
            cout << "\n=== Задача 2. Траектория ===" << endl;
            Trajectory t1;
            Trajectory t2(100, 200, 45);
            t1.printTrajectory();
            t2.printTrajectory();
            break;
        }

        case 3: {
            cout << "\n=== Задача 3. Двигатель ===" << endl;
            Engine e1(5000, 2.5);
            e1.printInfo();
            break;
        }

        case 4: {
            cout << "\n=== Задача 4. Гироскоп ===" << endl;
            Gyroscope g1(120.5, true);
            g1.printStatus();
            g1.calibrate();
            g1.printStatus();
            break;
        }

        case 5: {
            cout << "\n=== Задача 5. Автопилот ===" << endl;
            Autopilot a1(90, 1000);
            a1.changeCourse(10);
            a1.changeAltitude(500);
            a1.printStatus();
            break;
        }

        case 6: {
            cout << "\n=== Задача 6. Многоступенчатая ракета ===" << endl;
            RocketStage stage1(100000, 60, 1000);
            RocketStage stage2(80000, 45, 600);
            RocketStage stage3(50000, 30, 300);

            stage1.printInfo();
            stage2.printInfo();
            stage3.printInfo();

            double totalDeltaV = stage1.getDeltaV() + stage2.getDeltaV() + stage3.getDeltaV();
            cout << "Общая ΔV: " << totalDeltaV << " м/с" << endl;
            break;
        }

        case 7: {
            cout << "\n=== Задача 7. Управление полетом ===" << endl;
            FlightComputer fc(0, 0, 100, 20000);
            for (int i = 0; i < 3; i++) {
                fc.simulateStep(1.0);
                fc.printStatus();
            }
            break;
        }

        case 8: {
            cout << "\n=== Задача 8. Коррекция координат ===" << endl;
            NavigationSystem nav(0, 0, 100, 50, true);
            nav.integratePosition(1);
            nav.correctGPS(110, 60);
            nav.printPosition();
            break;
        }

        case 9: {
            cout << "\n=== Задача 9. Автономное управление высотой ===" << endl;
            AutonomousControl ac(0, 2000, 5000);
            for (int i = 0; i < 3; i++) {
                ac.updateControl();
                ac.simulateStep(1.0);
                ac.printStatus();
            }
            break;
        }

        case 10: {
            cout << "\n=== Задача 10. Анализ маршрута дрона ===" << endl;
            DroneFlight d;
            d.addPoint(0, 0);
            d.addPoint(3, 4);
            d.addPoint(6, 8);
            d.printPath();
            cout << "Пройдено: " << d.getTotalDistance() << " м" << endl;
            break;
        }

        case 11: {
            cout << "\n=== Задача 11. Комплексная система моделирования ===" << endl;
            Engine11 engine(100000, 50, 500);
            Navigation11 nav(0, 0, 1000);
            AutonomousFlightSystem afs(engine, nav);

            cout << "Начало симуляции:" << endl;
            afs.simulate(1.0, 10.0);
            afs.printSummary();
            break;
        }

        case 0: {
            cout << "Выход из программы." << endl;
            break;
        }

        default: {
            cout << "Неверный выбор. Попробуйте снова." << endl;
            break;
        }
        }

    } while (choice != 0);

    return 0;
}