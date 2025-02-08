/****************************************************************************
** Meta object code from reading C++ file 'agentgroup.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../../../src/pedsim_ros_imp/pedsim_simulator/include/pedsim_simulator/element/agentgroup.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'agentgroup.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_AgentGroup_t {
    QByteArrayData data[11];
    char stringdata0[110];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_AgentGroup_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_AgentGroup_t qt_meta_stringdata_AgentGroup = {
    {
QT_MOC_LITERAL(0, 0, 10), // "AgentGroup"
QT_MOC_LITERAL(1, 11, 14), // "membersChanged"
QT_MOC_LITERAL(2, 26, 0), // ""
QT_MOC_LITERAL(3, 27, 11), // "memberAdded"
QT_MOC_LITERAL(4, 39, 2), // "id"
QT_MOC_LITERAL(5, 42, 13), // "memberRemoved"
QT_MOC_LITERAL(6, 56, 17), // "onPositionChanged"
QT_MOC_LITERAL(7, 74, 1), // "x"
QT_MOC_LITERAL(8, 76, 1), // "y"
QT_MOC_LITERAL(9, 78, 18), // "updateCenterOfMass"
QT_MOC_LITERAL(10, 97, 12) // "Ped::Tvector"

    },
    "AgentGroup\0membersChanged\0\0memberAdded\0"
    "id\0memberRemoved\0onPositionChanged\0x\0"
    "y\0updateCenterOfMass\0Ped::Tvector"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_AgentGroup[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   39,    2, 0x06 /* Public */,
       3,    1,   40,    2, 0x06 /* Public */,
       5,    1,   43,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       6,    2,   46,    2, 0x0a /* Public */,
       9,    0,   51,    2, 0x09 /* Protected */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    4,
    QMetaType::Void, QMetaType::Int,    4,

 // slots: parameters
    QMetaType::Void, QMetaType::Double, QMetaType::Double,    7,    8,
    0x80000000 | 10,

       0        // eod
};

void AgentGroup::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<AgentGroup *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->membersChanged(); break;
        case 1: _t->memberAdded((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->memberRemoved((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 3: _t->onPositionChanged((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 4: { Ped::Tvector _r = _t->updateCenterOfMass();
            if (_a[0]) *reinterpret_cast< Ped::Tvector*>(_a[0]) = std::move(_r); }  break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (AgentGroup::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&AgentGroup::membersChanged)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (AgentGroup::*)(int );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&AgentGroup::memberAdded)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (AgentGroup::*)(int );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&AgentGroup::memberRemoved)) {
                *result = 2;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject AgentGroup::staticMetaObject = { {
    QMetaObject::SuperData::link<ScenarioElement::staticMetaObject>(),
    qt_meta_stringdata_AgentGroup.data,
    qt_meta_data_AgentGroup,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *AgentGroup::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *AgentGroup::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_AgentGroup.stringdata0))
        return static_cast<void*>(this);
    return ScenarioElement::qt_metacast(_clname);
}

int AgentGroup::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = ScenarioElement::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 5)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 5;
    }
    return _id;
}

// SIGNAL 0
void AgentGroup::membersChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}

// SIGNAL 1
void AgentGroup::memberAdded(int _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void AgentGroup::memberRemoved(int _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
