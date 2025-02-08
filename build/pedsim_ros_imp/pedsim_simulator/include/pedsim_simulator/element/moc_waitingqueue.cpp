/****************************************************************************
** Meta object code from reading C++ file 'waitingqueue.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../../../src/pedsim_ros_imp/pedsim_simulator/include/pedsim_simulator/element/waitingqueue.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'waitingqueue.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_WaitingQueue_t {
    QByteArrayData data[17];
    char stringdata0[192];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_WaitingQueue_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_WaitingQueue_t qt_meta_stringdata_WaitingQueue = {
    {
QT_MOC_LITERAL(0, 0, 12), // "WaitingQueue"
QT_MOC_LITERAL(1, 13, 16), // "directionChanged"
QT_MOC_LITERAL(2, 30, 0), // ""
QT_MOC_LITERAL(3, 31, 11), // "radianAngle"
QT_MOC_LITERAL(4, 43, 12), // "agentMayPass"
QT_MOC_LITERAL(5, 56, 2), // "id"
QT_MOC_LITERAL(6, 59, 13), // "agentDequeued"
QT_MOC_LITERAL(7, 73, 18), // "queueLeaderChanged"
QT_MOC_LITERAL(8, 92, 15), // "queueEndChanged"
QT_MOC_LITERAL(9, 108, 23), // "queueEndPositionChanged"
QT_MOC_LITERAL(10, 132, 1), // "x"
QT_MOC_LITERAL(11, 134, 1), // "y"
QT_MOC_LITERAL(12, 136, 13), // "onTimeChanged"
QT_MOC_LITERAL(13, 150, 6), // "timeIn"
QT_MOC_LITERAL(14, 157, 26), // "onLastAgentPositionChanged"
QT_MOC_LITERAL(15, 184, 3), // "xIn"
QT_MOC_LITERAL(16, 188, 3) // "yIn"

    },
    "WaitingQueue\0directionChanged\0\0"
    "radianAngle\0agentMayPass\0id\0agentDequeued\0"
    "queueLeaderChanged\0queueEndChanged\0"
    "queueEndPositionChanged\0x\0y\0onTimeChanged\0"
    "timeIn\0onLastAgentPositionChanged\0xIn\0"
    "yIn"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_WaitingQueue[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       6,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   54,    2, 0x06 /* Public */,
       4,    1,   57,    2, 0x06 /* Public */,
       6,    1,   60,    2, 0x06 /* Public */,
       7,    1,   63,    2, 0x06 /* Public */,
       8,    0,   66,    2, 0x06 /* Public */,
       9,    2,   67,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      12,    1,   72,    2, 0x09 /* Protected */,
      14,    2,   75,    2, 0x09 /* Protected */,

 // signals: parameters
    QMetaType::Void, QMetaType::Double,    3,
    QMetaType::Void, QMetaType::Int,    5,
    QMetaType::Void, QMetaType::Int,    5,
    QMetaType::Void, QMetaType::Int,    5,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Double, QMetaType::Double,   10,   11,

 // slots: parameters
    QMetaType::Void, QMetaType::Double,   13,
    QMetaType::Void, QMetaType::Double, QMetaType::Double,   15,   16,

       0        // eod
};

void WaitingQueue::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<WaitingQueue *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->directionChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 1: _t->agentMayPass((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->agentDequeued((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 3: _t->queueLeaderChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 4: _t->queueEndChanged(); break;
        case 5: _t->queueEndPositionChanged((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 6: _t->onTimeChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 7: _t->onLastAgentPositionChanged((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (WaitingQueue::*)(double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&WaitingQueue::directionChanged)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (WaitingQueue::*)(int );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&WaitingQueue::agentMayPass)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (WaitingQueue::*)(int );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&WaitingQueue::agentDequeued)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (WaitingQueue::*)(int );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&WaitingQueue::queueLeaderChanged)) {
                *result = 3;
                return;
            }
        }
        {
            using _t = void (WaitingQueue::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&WaitingQueue::queueEndChanged)) {
                *result = 4;
                return;
            }
        }
        {
            using _t = void (WaitingQueue::*)(double , double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&WaitingQueue::queueEndPositionChanged)) {
                *result = 5;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject WaitingQueue::staticMetaObject = { {
    QMetaObject::SuperData::link<Waypoint::staticMetaObject>(),
    qt_meta_stringdata_WaitingQueue.data,
    qt_meta_data_WaitingQueue,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *WaitingQueue::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *WaitingQueue::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_WaitingQueue.stringdata0))
        return static_cast<void*>(this);
    return Waypoint::qt_metacast(_clname);
}

int WaitingQueue::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = Waypoint::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 8)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 8;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 8)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 8;
    }
    return _id;
}

// SIGNAL 0
void WaitingQueue::directionChanged(double _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void WaitingQueue::agentMayPass(int _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void WaitingQueue::agentDequeued(int _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void WaitingQueue::queueLeaderChanged(int _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void WaitingQueue::queueEndChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 4, nullptr);
}

// SIGNAL 5
void WaitingQueue::queueEndPositionChanged(double _t1, double _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
