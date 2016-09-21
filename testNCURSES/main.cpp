#include <iostream>
#include <dlfcn.h> //Ubuntu
#include <KinovaTypes.h>
#include <Kinova.API.CommLayerUbuntu.h>
#include <Kinova.API.UsbCommandLayerUbuntu.h>
#include <unistd.h>  // for sleep
#include <ncurses.h> // for keyboard events

using namespace std;

int kbhit(void)
{
    int ch = getch();

    if (ch != ERR) {
        ungetch(ch);
        return 1;
    } else {
        return 0;
    }
}



int main(void)
{
    cout << endl << endl;
    cout << "========================================================" << endl;
    cout << "=====  Testing keyboard Event handling             =====" << endl;
    cout << "========================================================" << endl;
    cout << "code: Reza Ahmadzadeh (IRIM, 2016)." << endl;
    cout << "WARNING: Read the documents before running this code!" << endl << endl;


    /* test1 */

    initscr();
    cbreak();
    noecho();
    nodelay(stdscr, TRUE);
    scrollok(stdscr, TRUE);
    int ch, dill;
    dill = 0;
    while (dill<1000 && ch!='s') {
        if (kbhit()) {
            ch = getch();
            printw("Key pressed! It was: %d\n", ch);
            refresh();
            sleep(1);
        } else {
            printw("No key pressed yet...\n");
            refresh();
            dill += 1;
            printw("dill %d\n",dill);
            sleep(1);
        }
    }
    /**/

    /* test2
    initscr();
    refresh();
    getch();
    endwin();
    */


//    int ch;

//        initscr();			/* Start curses mode 		*/
//        raw();				/* Line buffering disabled	*/
//        keypad(stdscr, TRUE);		/* We get F1, F2 etc..		*/
//        noecho();			/* Don't echo() while we do getch */

//            printw("Type any character to see it in bold\n");
//        ch = getch();			/* If raw() hadn't been called
//                         * we have to press enter before it
//                         * gets to the program 		*/
//        if(ch == KEY_F(1))		/* Without keypad enabled this will */
//            printw("F1 Key pressed");/*  not get to us either	*/
//                        /* Without noecho() some ugly escape
//                         * charachters might have been printed
//                         * on screen			*/
//        else
//        {	printw("The pressed key is ");
//            attron(A_BOLD);
//            printw("%c", ch);
//            attroff(A_BOLD);
//        }
//        refresh();			/* Print it on to the real screen */
//        getch();			/* Wait for user input */
//        endwin();			/* End curses mode		  */

//    return 0;
//
}

