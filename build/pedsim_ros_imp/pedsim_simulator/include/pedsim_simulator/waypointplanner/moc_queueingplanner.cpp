/****************************************************************************
** Meta object code from reading C++ file 'queueingplanner.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../../../src/pedsim_ros_imp/pedsim_simulator/include/pedsim_simulator/waypointplanner/queueingplanner.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'queueingplanner.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_QueueingWaypointPlanner_t {
    QByteArrayData data[9];
    char stringdata0[138];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_QueueingWaypointPlanner_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_QueueingWaypointPlanner_t qt_meta_stringdata_QueueingWaypointPlanner = {
    {
QT_MOC_LITERAL(0, 0, 23), // "QueueingWaypointPlanner"
QT_MOC_LITERAL(1, 24, 30), // "onFollowedAgentPositionChanged"
QT_MOC_LITERAL(2, 55, 0), // ""
QT_MOC_LITERAL(3, 56, 3), // "xIn"
QT_MOC_LITERAL(4, 60, 3), // "yIn"
QT_MOC_LITERAL(5, 64, 19), // "onAgentMayPassQueue"
QT_MOC_LITERAL(6, 84, 2), // "id"
QT_MOC_LITERAL(7, 87, 24), // "onFollowedAgentLeftQueue"
QT_MOC_LITERAL(8, 112, 25) // "onQueueEndPositionChanged"

    },
    "QueueingWaypointPlanner\0"
    "onFollowedAgentPositionChanged\0\0xIn\0"
    "yIn\0onAgentMayPassQueue\0id\0"
    "onFollowedAgentLeftQueue\0"
    "onQueueEndPositionChanged"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_QueueingWaypointPlanner[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    2,   34,    2, 0x09 /* Protected */,
       5,    1,   39,    2, 0x09 /* Protected */,
       7,    0,   42,    2, 0x09 /* Protected */,
       8,    2,   43,    2, 0x09 /* Protected */,

 // slots: parameters
    QMetaType::Void, QMetaType::Double, QMetaType::Double,    3,    4,
    QMetaType::Void, QMetaType::Int,    6,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Double, QMetaType::Double,    3,    4,

       0        // eod
};

void QueueingWaypointPlanner::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<QueueingWaypointPlanner *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->onFollowedAgentPositionChanged((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 1: _t->onAgentMayPassQueue((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->onFollowedAgentLeftQueue(); break;
        case 3: _t->onQueueEndPositionChanged((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject QueueingWaypointPlanner::staticMetaObject = { {
    QMetaObject::SuperData::link<WaypointPlanner::staticMetaObject>(),
    qt_meta_stringdata_QueueingWaypointPlanner.data,
    qt_meta_data_QueueingWaypointPlanner,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *QueueingWaypointPlanner::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *QueueingWaypointPlanner::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_QueueingWaypointPlanner.stringdata0))
        return static_cast<void*>(this);
    return WaypointPlanner::qt_metacast(_clname);
}

int QueueingWaypointPlanner::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = WaypointPlanner::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 4)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 4;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
