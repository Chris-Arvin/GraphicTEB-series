/****************************************************************************
** Meta object code from reading C++ file 'attractionarea.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../../../src/pedsim_ros_imp/pedsim_simulator/include/pedsim_simulator/element/attractionarea.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'attractionarea.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_AttractionArea_t {
    QByteArrayData data[10];
    char stringdata0[86];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_AttractionArea_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_AttractionArea_t qt_meta_stringdata_AttractionArea = {
    {
QT_MOC_LITERAL(0, 0, 14), // "AttractionArea"
QT_MOC_LITERAL(1, 15, 15), // "positionChanged"
QT_MOC_LITERAL(2, 31, 0), // ""
QT_MOC_LITERAL(3, 32, 1), // "x"
QT_MOC_LITERAL(4, 34, 1), // "y"
QT_MOC_LITERAL(5, 36, 11), // "sizeChanged"
QT_MOC_LITERAL(6, 48, 5), // "width"
QT_MOC_LITERAL(7, 54, 6), // "height"
QT_MOC_LITERAL(8, 61, 15), // "strengthChanged"
QT_MOC_LITERAL(9, 77, 8) // "strength"

    },
    "AttractionArea\0positionChanged\0\0x\0y\0"
    "sizeChanged\0width\0height\0strengthChanged\0"
    "strength"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_AttractionArea[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    2,   29,    2, 0x06 /* Public */,
       5,    2,   34,    2, 0x06 /* Public */,
       8,    1,   39,    2, 0x06 /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::Double, QMetaType::Double,    3,    4,
    QMetaType::Void, QMetaType::Double, QMetaType::Double,    6,    7,
    QMetaType::Void, QMetaType::Double,    9,

       0        // eod
};

void AttractionArea::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<AttractionArea *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->positionChanged((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 1: _t->sizeChanged((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 2: _t->strengthChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (AttractionArea::*)(double , double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&AttractionArea::positionChanged)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (AttractionArea::*)(double , double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&AttractionArea::sizeChanged)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (AttractionArea::*)(double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&AttractionArea::strengthChanged)) {
                *result = 2;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject AttractionArea::staticMetaObject = { {
    QMetaObject::SuperData::link<ScenarioElement::staticMetaObject>(),
    qt_meta_stringdata_AttractionArea.data,
    qt_meta_data_AttractionArea,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *AttractionArea::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *AttractionArea::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_AttractionArea.stringdata0))
        return static_cast<void*>(this);
    return ScenarioElement::qt_metacast(_clname);
}

int AttractionArea::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = ScenarioElement::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 3)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 3;
    }
    return _id;
}

// SIGNAL 0
void AttractionArea::positionChanged(double _t1, double _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void AttractionArea::sizeChanged(double _t1, double _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void AttractionArea::strengthChanged(double _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
