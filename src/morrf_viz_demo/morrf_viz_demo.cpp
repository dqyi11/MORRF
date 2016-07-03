#include <QApplication>
#include <iostream>
#include "morrf_viz_demo_cmdline.h"
#include "mainwindow.h"

int main(int argc, char *argv[]) {

    gengetopt_args_info args;
    if( cmdline_parser( argc, argv, &args ) != 0 ){
        exit(1);
    }

    QApplication a(argc, argv);
    MainWindow w;
    if( args.no_gui_arg > 0  ) {
        w.hide();
        QString config_filename(args.config_arg);
        QString paths_filename(args.paths_arg);
        QString log_filename(args.log_arg);
        std::cout << "loading " << config_filename.toStdString() << std::endl;
        if(w.planPath(config_filename, paths_filename, log_filename )) {
            std::cout << "saving to " << paths_filename.toStdString() << std::endl;
            std::cout << "logging to " << log_filename.toStdString() << std::endl;
        }
        return 0;
    }

    w.show();
    return a.exec();
}
