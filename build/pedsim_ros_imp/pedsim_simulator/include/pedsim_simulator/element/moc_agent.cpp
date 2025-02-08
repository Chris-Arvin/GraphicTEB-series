/****************************************************************************
** Meta object code from reading C++ file 'agent.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../../../src/pedsim_ros_imp/pedsim_simulator/include/pedsim_simulator/element/agent.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'agent.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_Agent_t {
    QByteArrayData data[18];
    char stringdata0[223];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_Agent_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_Agent_t qt_meta_stringdata_Agent = {
    {
QT_MOC_LITERAL(0, 0, 5), // "Agent"
QT_MOC_LITERAL(1, 6, 15), // "positionChanged"
QT_MOC_LITERAL(2, 22, 0), // ""
QT_MOC_LITERAL(3, 23, 1), // "x"
QT_MOC_LITERAL(4, 25, 1), // "y"
QT_MOC_LITERAL(5, 27, 15), // "velocityChanged"
QT_MOC_LITERAL(6, 43, 19), // "accelerationChanged"
QT_MOC_LITERAL(7, 63, 19), // "desiredForceChanged"
QT_MOC_LITERAL(8, 83, 20), // "obstacleForceChanged"
QT_MOC_LITERAL(9, 104, 18), // "socialForceChanged"
QT_MOC_LITERAL(10, 123, 14), // "myForceChanged"
QT_MOC_LITERAL(11, 138, 22), // "additionalForceChanged"
QT_MOC_LITERAL(12, 161, 4), // "name"
QT_MOC_LITERAL(13, 166, 15), // "reachedWaypoint"
QT_MOC_LITERAL(14, 182, 11), // "typeChanged"
QT_MOC_LITERAL(15, 194, 4), // "type"
QT_MOC_LITERAL(16, 199, 10), // "forceAdded"
QT_MOC_LITERAL(17, 210, 12) // "forceRemoved"

    },
    "Agent\0positionChanged\0\0x\0y\0velocityChanged\0"
    "accelerationChanged\0desiredForceChanged\0"
    "obstacleForceChanged\0socialForceChanged\0"
    "myForceChanged\0additionalForceChanged\0"
    "name\0reachedWaypoint\0typeChanged\0type\0"
    "forceAdded\0forceRemoved"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_Agent[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      12,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
      12,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    2,   74,    2, 0x06 /* Public */,
       5,    2,   79,    2, 0x06 /* Public */,
       6,    2,   84,    2, 0x06 /* Public */,
       7,    2,   89,    2, 0x06 /* Public */,
       8,    2,   94,    2, 0x06 /* Public */,
       9,    2,   99,    2, 0x06 /* Public */,
      10,    2,  104,    2, 0x06 /* Public */,
      11,    3,  109,    2, 0x06 /* Public */,
      13,    0,  116,    2, 0x06 /* Public */,
      14,    1,  117,    2, 0x06 /* Public */,
      16,    1,  120,    2, 0x06 /* Public */,
      17,    1,  123,    2, 0x06 /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::Double, QMetaType::Double,    3,    4,
    QMetaType::Void, QMetaType::Double, QMetaType::Double,    3,    4,
    QMetaType::Void, QMetaType::Double, QMetaType::Double,    3,    4,
    QMetaType::Void, QMetaType::Double, QMetaType::Double,    3,    4,
    QMetaType::Void, QMetaType::Double, QMetaType::Double,    3,    4,
    QMetaType::Void, QMetaType::Double, QMetaType::Double,    3,    4,
    QMetaType::Void, QMetaType::Double, QMetaType::Double,    3,    4,
    QMetaType::Void, QMetaType::QString, QMetaType::Double, QMetaType::Double,   12,    3,    4,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,   15,
    QMetaType::Void, QMetaType::QString,   12,
    QMetaType::Void, QMetaType::QString,   12,

       0        // eod
};

void Agent::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<Agent *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->positionChanged((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 1: _t->velocityChanged((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 2: _t->accelerationChanged((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 3: _t->desiredForceChanged((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 4: _t->obstacleForceChanged((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 5: _t->socialForceChanged((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 6: _t->myForceChanged((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 7: _t->additionalForceChanged((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3]))); break;
        case 8: _t->reachedWaypoint(); break;
        case 9: _t->typeChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 10: _t->forceAdded((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 11: _t->forceRemoved((*reinterpret_cast< QString(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (Agent::*)(double , double ) const;
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Agent::positionChanged)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (Agent::*)(double , double ) const;
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Agent::velocityChanged)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (Agent::*)(double , double ) const;
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Agent::accelerationChanged)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (Agent::*)(double , double ) const;
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Agent::desiredForceChanged)) {
                *result = 3;
                return;
            }
        }
        {
            using _t = void (Agent::*)(double , double ) const;
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Agent::obstacleForceChanged)) {
                *result = 4;
                return;
            }
        }
        {
            using _t = void (Agent::*)(double , double ) const;
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Agent::socialForceChanged)) {
                *result = 5;
                return;
            }
        }
        {
            using _t = void (Agent::*)(double , double ) const;
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Agent::myForceChanged)) {
                *result = 6;
                return;
            }
        }
        {
            using _t = void (Agent::*)(QString , double , double ) const;
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Agent::additionalForceChanged)) {
                *result = 7;
                return;
            }
        }
        {
            using _t = void (Agent::*)() const;
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Agent::reachedWaypoint)) {
                *result = 8;
                return;
            }
        }
        {
            using _t = void (Agent::*)(int );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Agent::typeChanged)) {
                *result = 9;
                return;
            }
        }
        {
            using _t = void (Agent::*)(QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Agent::forceAdded)) {
                *result = 10;
                return;
            }
        }
        {
            using _t = void (Agent::*)(QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Agent::forceRemoved)) {
                *result = 11;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject Agent::staticMetaObject = { {
    QMetaObject::SuperData::link<ScenarioElement::staticMetaObject>(),
    qt_meta_stringdata_Agent.data,
    qt_meta_data_Agent,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *Agent::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *Agent::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_Agent.stringdata0))
        return static_cast<void*>(this);
    if (!strcmp(_clname, "Ped::Tagent"))
        return static_cast< Ped::Tagent*>(this);
    return ScenarioElement::qt_metacast(_clname);
}

int Agent::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = ScenarioElement::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 12)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 12;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 12)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 12;
    }
    return _id;
}

// SIGNAL 0
void Agent::positionChanged(double _t1, double _t2)const
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(const_cast< Agent *>(this), &staticMetaObject, 0, _a);
}

// SIGNAL 1
void Agent::velocityChanged(double _t1, double _t2)const
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(const_cast< Agent *>(this), &staticMetaObject, 1, _a);
}

// SIGNAL 2
void Agent::accelerationChanged(double _t1, double _t2)const
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(const_cast< Agent *>(this), &staticMetaObject, 2, _a);
}

// SIGNAL 3
void Agent::desiredForceChanged(double _t1, double _t2)const
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(const_cast< Agent *>(this), &staticMetaObject, 3, _a);
}

// SIGNAL 4
void Agent::obstacleForceChanged(double _t1, double _t2)const
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(const_cast< Agent *>(this), &staticMetaObject, 4, _a);
}

// SIGNAL 5
void Agent::socialForceChanged(double _t1, double _t2)const
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(const_cast< Agent *>(this), &staticMetaObject, 5, _a);
}

// SIGNAL 6
void Agent::myForceChanged(double _t1, double _t2)const
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(const_cast< Agent *>(this), &staticMetaObject, 6, _a);
}

// SIGNAL 7
void Agent::additionalForceChanged(QString _t1, double _t2, double _t3)const
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t3))) };
    QMetaObject::activate(const_cast< Agent *>(this), &staticMetaObject, 7, _a);
}

// SIGNAL 8
void Agent::reachedWaypoint()const
{
    QMetaObject::activate(const_cast< Agent *>(this), &staticMetaObject, 8, nullptr);
}

// SIGNAL 9
void Agent::typeChanged(int _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 9, _a);
}

// SIGNAL 10
void Agent::forceAdded(QString _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 10, _a);
}

// SIGNAL 11
void Agent::forceRemoved(QString _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 11, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
