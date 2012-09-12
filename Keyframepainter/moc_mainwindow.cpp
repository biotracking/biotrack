/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created: Mon Oct 3 15:13:55 2011
**      by: The Qt Meta Object Compiler version 62 (Qt 4.7.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "mainwindow.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.7.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_MainWindow[] = {

 // content:
       5,       // revision
       0,       // classname
       0,    0, // classinfo
      10,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      12,   11,   11,   11, 0x0a,
      38,   11,   11,   11, 0x0a,
      62,   11,   11,   11, 0x0a,
      86,   11,   11,   11, 0x0a,
     111,   11,   11,   11, 0x0a,
     139,   11,   11,   11, 0x0a,
     163,   11,   11,   11, 0x0a,
     198,   11,   11,   11, 0x0a,
     223,   11,   11,   11, 0x0a,
     252,   11,   11,   11, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_MainWindow[] = {
    "MainWindow\0\0on_actionExit_triggered()\0"
    "on_stopButton_clicked()\0on_playButton_clicked()\0"
    "on_targetpaint_clicked()\0"
    "on_nottargetpaint_clicked()\0"
    "on_pushButton_clicked()\0"
    "on_previousFrameButton_2_clicked()\0"
    "on_clearButton_clicked()\0"
    "on_saveFrameButton_clicked()\0"
    "on_backgroundmaskbutton_clicked()\0"
};

const QMetaObject MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_MainWindow,
      qt_meta_data_MainWindow, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &MainWindow::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: on_actionExit_triggered(); break;
        case 1: on_stopButton_clicked(); break;
        case 2: on_playButton_clicked(); break;
        case 3: on_targetpaint_clicked(); break;
        case 4: on_nottargetpaint_clicked(); break;
        case 7: on_clearButton_clicked(); break;
        case 8: on_saveFrameButton_clicked(); break;
        case 9: on_backgroundmaskbutton_clicked(); break;
        default: ;
        }
        _id -= 10;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
