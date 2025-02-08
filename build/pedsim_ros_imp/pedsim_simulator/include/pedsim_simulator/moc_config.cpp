/****************************************************************************
** Meta object code from reading C++ file 'config.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../../src/pedsim_ros_imp/pedsim_simulator/include/pedsim_simulator/config.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'config.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_Config_t {
    QByteArrayData data[22];
    char stringdata0[417];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_Config_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_Config_t qt_meta_stringdata_Config = {
    {
QT_MOC_LITERAL(0, 0, 6), // "Config"
QT_MOC_LITERAL(1, 7, 18), // "forceFactorChanged"
QT_MOC_LITERAL(2, 26, 0), // ""
QT_MOC_LITERAL(3, 27, 4), // "name"
QT_MOC_LITERAL(4, 32, 5), // "value"
QT_MOC_LITERAL(5, 38, 26), // "forceFactorObstacleChanged"
QT_MOC_LITERAL(6, 65, 25), // "forceSigmaObstacleChanged"
QT_MOC_LITERAL(7, 91, 24), // "forceFactorSocialChanged"
QT_MOC_LITERAL(8, 116, 27), // "forceFactorGroupGazeChanged"
QT_MOC_LITERAL(9, 144, 32), // "forceFactorGroupCoherenceChanged"
QT_MOC_LITERAL(10, 177, 32), // "forceFactorGroupRepulsionChanged"
QT_MOC_LITERAL(11, 210, 24), // "forceFactorRandomChanged"
QT_MOC_LITERAL(12, 235, 27), // "forceFactorAlongWallChanged"
QT_MOC_LITERAL(13, 263, 16), // "setObstacleForce"
QT_MOC_LITERAL(14, 280, 7), // "valueIn"
QT_MOC_LITERAL(15, 288, 16), // "setObstacleSigma"
QT_MOC_LITERAL(16, 305, 14), // "setSocialForce"
QT_MOC_LITERAL(17, 320, 17), // "setGroupGazeForce"
QT_MOC_LITERAL(18, 338, 22), // "setGroupCoherenceForce"
QT_MOC_LITERAL(19, 361, 22), // "setGroupRepulsionForce"
QT_MOC_LITERAL(20, 384, 14), // "setRandomForce"
QT_MOC_LITERAL(21, 399, 17) // "setAlongWallForce"

    },
    "Config\0forceFactorChanged\0\0name\0value\0"
    "forceFactorObstacleChanged\0"
    "forceSigmaObstacleChanged\0"
    "forceFactorSocialChanged\0"
    "forceFactorGroupGazeChanged\0"
    "forceFactorGroupCoherenceChanged\0"
    "forceFactorGroupRepulsionChanged\0"
    "forceFactorRandomChanged\0"
    "forceFactorAlongWallChanged\0"
    "setObstacleForce\0valueIn\0setObstacleSigma\0"
    "setSocialForce\0setGroupGazeForce\0"
    "setGroupCoherenceForce\0setGroupRepulsionForce\0"
    "setRandomForce\0setAlongWallForce"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_Config[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      17,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       9,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    2,   99,    2, 0x06 /* Public */,
       5,    1,  104,    2, 0x06 /* Public */,
       6,    1,  107,    2, 0x06 /* Public */,
       7,    1,  110,    2, 0x06 /* Public */,
       8,    1,  113,    2, 0x06 /* Public */,
       9,    1,  116,    2, 0x06 /* Public */,
      10,    1,  119,    2, 0x06 /* Public */,
      11,    1,  122,    2, 0x06 /* Public */,
      12,    1,  125,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      13,    1,  128,    2, 0x0a /* Public */,
      15,    1,  131,    2, 0x0a /* Public */,
      16,    1,  134,    2, 0x0a /* Public */,
      17,    1,  137,    2, 0x0a /* Public */,
      18,    1,  140,    2, 0x0a /* Public */,
      19,    1,  143,    2, 0x0a /* Public */,
      20,    1,  146,    2, 0x0a /* Public */,
      21,    1,  149,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::QString, QMetaType::Double,    3,    4,
    QMetaType::Void, QMetaType::Double,    4,
    QMetaType::Void, QMetaType::Double,    4,
    QMetaType::Void, QMetaType::Double,    4,
    QMetaType::Void, QMetaType::Double,    4,
    QMetaType::Void, QMetaType::Double,    4,
    QMetaType::Void, QMetaType::Double,    4,
    QMetaType::Void, QMetaType::Double,    4,
    QMetaType::Void, QMetaType::Double,    4,

 // slots: parameters
    QMetaType::Void, QMetaType::Double,   14,
    QMetaType::Void, QMetaType::Double,   14,
    QMetaType::Void, QMetaType::Double,   14,
    QMetaType::Void, QMetaType::Double,   14,
    QMetaType::Void, QMetaType::Double,   14,
    QMetaType::Void, QMetaType::Double,   14,
    QMetaType::Void, QMetaType::Double,   14,
    QMetaType::Void, QMetaType::Double,   14,

       0        // eod
};

void Config::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<Config *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->forceFactorChanged((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 1: _t->forceFactorObstacleChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 2: _t->forceSigmaObstacleChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 3: _t->forceFactorSocialChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 4: _t->forceFactorGroupGazeChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 5: _t->forceFactorGroupCoherenceChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 6: _t->forceFactorGroupRepulsionChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 7: _t->forceFactorRandomChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 8: _t->forceFactorAlongWallChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 9: _t->setObstacleForce((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 10: _t->setObstacleSigma((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 11: _t->setSocialForce((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 12: _t->setGroupGazeForce((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 13: _t->setGroupCoherenceForce((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 14: _t->setGroupRepulsionForce((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 15: _t->setRandomForce((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 16: _t->setAlongWallForce((*reinterpret_cast< double(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (Config::*)(QString , double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Config::forceFactorChanged)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (Config::*)(double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Config::forceFactorObstacleChanged)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (Config::*)(double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Config::forceSigmaObstacleChanged)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (Config::*)(double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Config::forceFactorSocialChanged)) {
                *result = 3;
                return;
            }
        }
        {
            using _t = void (Config::*)(double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Config::forceFactorGroupGazeChanged)) {
                *result = 4;
                return;
            }
        }
        {
            using _t = void (Config::*)(double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Config::forceFactorGroupCoherenceChanged)) {
                *result = 5;
                return;
            }
        }
        {
            using _t = void (Config::*)(double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Config::forceFactorGroupRepulsionChanged)) {
                *result = 6;
                return;
            }
        }
        {
            using _t = void (Config::*)(double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Config::forceFactorRandomChanged)) {
                *result = 7;
                return;
            }
        }
        {
            using _t = void (Config::*)(double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Config::forceFactorAlongWallChanged)) {
                *result = 8;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject Config::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_Config.data,
    qt_meta_data_Config,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *Config::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *Config::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_Config.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int Config::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 17)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 17;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 17)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 17;
    }
    return _id;
}

// SIGNAL 0
void Config::forceFactorChanged(QString _t1, double _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void Config::forceFactorObstacleChanged(double _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void Config::forceSigmaObstacleChanged(double _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void Config::forceFactorSocialChanged(double _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void Config::forceFactorGroupGazeChanged(double _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void Config::forceFactorGroupCoherenceChanged(double _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}

// SIGNAL 6
void Config::forceFactorGroupRepulsionChanged(double _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 6, _a);
}

// SIGNAL 7
void Config::forceFactorRandomChanged(double _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 7, _a);
}

// SIGNAL 8
void Config::forceFactorAlongWallChanged(double _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 8, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
