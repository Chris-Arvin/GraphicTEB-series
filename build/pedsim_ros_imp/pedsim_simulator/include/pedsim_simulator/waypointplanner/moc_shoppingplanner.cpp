/****************************************************************************
** Meta object code from reading C++ file 'shoppingplanner.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../../../src/pedsim_ros_imp/pedsim_simulator/include/pedsim_simulator/waypointplanner/shoppingplanner.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'shoppingplanner.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_ShoppingPlanner_t {
    QByteArrayData data[4];
    char stringdata0[47];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_ShoppingPlanner_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_ShoppingPlanner_t qt_meta_stringdata_ShoppingPlanner = {
    {
QT_MOC_LITERAL(0, 0, 15), // "ShoppingPlanner"
QT_MOC_LITERAL(1, 16, 14), // "lostAttraction"
QT_MOC_LITERAL(2, 31, 0), // ""
QT_MOC_LITERAL(3, 32, 14) // "loseAttraction"

    },
    "ShoppingPlanner\0lostAttraction\0\0"
    "loseAttraction"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_ShoppingPlanner[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   24,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       3,    0,   25,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void,

       0        // eod
};

void ShoppingPlanner::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<ShoppingPlanner *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->lostAttraction(); break;
        case 1: _t->loseAttraction(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (ShoppingPlanner::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ShoppingPlanner::lostAttraction)) {
                *result = 0;
                return;
            }
        }
    }
    (void)_a;
}

QT_INIT_METAOBJECT const QMetaObject ShoppingPlanner::staticMetaObject = { {
    QMetaObject::SuperData::link<WaypointPlanner::staticMetaObject>(),
    qt_meta_stringdata_ShoppingPlanner.data,
    qt_meta_data_ShoppingPlanner,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *ShoppingPlanner::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ShoppingPlanner::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_ShoppingPlanner.stringdata0))
        return static_cast<void*>(this);
    return WaypointPlanner::qt_metacast(_clname);
}

int ShoppingPlanner::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = WaypointPlanner::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 2)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 2;
    }
    return _id;
}

// SIGNAL 0
void ShoppingPlanner::lostAttraction()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
