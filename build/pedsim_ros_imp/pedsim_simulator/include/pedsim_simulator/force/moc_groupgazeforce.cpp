/****************************************************************************
** Meta object code from reading C++ file 'groupgazeforce.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../../../src/pedsim_ros_imp/pedsim_simulator/include/pedsim_simulator/force/groupgazeforce.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'groupgazeforce.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_GroupGazeForce_t {
    QByteArrayData data[4];
    char stringdata0[54];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_GroupGazeForce_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_GroupGazeForce_t qt_meta_stringdata_GroupGazeForce = {
    {
QT_MOC_LITERAL(0, 0, 14), // "GroupGazeForce"
QT_MOC_LITERAL(1, 15, 29), // "onForceFactorGroupGazeChanged"
QT_MOC_LITERAL(2, 45, 0), // ""
QT_MOC_LITERAL(3, 46, 7) // "valueIn"

    },
    "GroupGazeForce\0onForceFactorGroupGazeChanged\0"
    "\0valueIn"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_GroupGazeForce[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       1,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   19,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void, QMetaType::Double,    3,

       0        // eod
};

void GroupGazeForce::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<GroupGazeForce *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->onForceFactorGroupGazeChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject GroupGazeForce::staticMetaObject = { {
    QMetaObject::SuperData::link<Force::staticMetaObject>(),
    qt_meta_stringdata_GroupGazeForce.data,
    qt_meta_data_GroupGazeForce,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *GroupGazeForce::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *GroupGazeForce::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_GroupGazeForce.stringdata0))
        return static_cast<void*>(this);
    return Force::qt_metacast(_clname);
}

int GroupGazeForce::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = Force::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 1)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 1;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 1)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 1;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
