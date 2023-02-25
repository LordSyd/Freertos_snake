/* llist.h
 * Generic Linked List
 */

struct node {
    int display;
    int x;
    int y;
    struct node *next;
};

typedef struct node * llist;

/* llist_create: Create a linked list */
llist *llist_create(void *data);

/* llist_free: Free a linked list */
void llist_free(llist *list);



/* llist_push: Add to head of list */
void llist_push(llist *list, int display, int y, int x /*void *data*/);

/* llist_print: print linked list */
void llist_print(llist *list, void (*print)(void *data));

void llist_printSnake(llist *list, int board[][8][8]);
void llist_setZero(llist *list, int board[][8][8]);

void moveRight(llist *list, int grow);
void moveLeft(llist *list, int grow);
void moveUp(llist *list, int grow);
void moveDown(llist *list, int grow);

int checkOccupiedRight(llist *list, int boardState[][8][8]);
int checkOccupiedLeft(llist *list, int boardState[][8][8]);
int checkOccupiedUp(llist *list, int boardState[][8][8]);
int checkOccupiedDown(llist *list, int boardState[][8][8]);
