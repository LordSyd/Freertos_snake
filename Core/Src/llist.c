/* llist.c
 * Generic Linked List implementation
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "llist.h"

llist *llist_create(void *new_data)
{
    struct node *new_node;

    llist *new_list = (llist *)malloc(sizeof (llist));
    *new_list = (struct node *)malloc(sizeof (struct node));


    new_node = *new_list;
    new_node->display = 0;
    new_node->x = 0;
    new_node->y = 4;
    new_node->next = NULL;
    return new_list;
}

void llist_free(llist *list)
{
    struct node *curr = *list;
    struct node *next;

    while (curr != NULL) {
        next = curr->next;
        free(curr);
        curr = next;
    }

    free(list);
}

// Returns 0 on failure


void llist_push(llist *list, int display, int y, int x /*void *data*/)
{
    struct node *head;
    struct node *new_node;
    if (list == NULL || *list == NULL) {
        fprintf(stderr, "llist_add_inorder: list is null\n");
    }

    head = *list;

    // Head is empty node


    // Head is not empty, add new node to front



        new_node = malloc(sizeof (struct node));
        new_node->display = display;
        new_node->x = x;
        new_node->y = y;
        new_node->next = head;
        *list = new_node;

}


void llist_printSnake(llist *list, int board[][8][8]) {

	 struct node *curr = *list;
	 int display ;
	int x;
	int y;


	    while (curr != NULL) {
	    	int display = curr->display;
			int x = curr->x;
			int y = curr->y;


	    	board[display][y][x] = 1;

	        curr = curr->next;
	    }

}

void llist_setZero(llist *list, int board[][8][8]) {

	 struct node *curr = *list;

	    while (curr != NULL) {

	    	int currBoard = board[curr->display][curr->x][curr->y];

	    	board[curr->display][curr->y][curr->x] = 0;

	        curr = curr->next;
	    }

}




int checkOccupiedRight(llist *list, int boardState[][8][8]) {
	struct node *head = *list;
	int nextCellState = 0;
	int display = head->display;
	int x = head->x;
	int y = head->y;
	int temp[4][8][8];
	memcpy(temp, boardState, sizeof(temp));
	if (x == 7){
		 if (display == 3) return -1;// ran into wall


		 nextCellState = temp[display+1][y][0];

	} else {
		nextCellState = temp[display][y][x+1];//todo change all to +1 and -1


	}


	return nextCellState;
}

int checkOccupiedLeft(llist *list, int boardState[][8][8]) {
	struct node *head = *list;

	int nextCellState=0;
	int display = head->display;
		int x = head->x;
		int y = head->y;
	int temp[4][8][8];
		memcpy(temp, boardState, sizeof(temp));
	if (head->x == 0){
		 if (head->display == 0) return -1;// ran into wall

		 nextCellState = temp[display-1][y][7];

	} else {

		nextCellState = temp[display][y][x-1];

	}


	return nextCellState;
}

int checkOccupiedUp(llist *list, int boardState[][8][8]) {
	struct node *head = *list;
	int display = head->display;
		int x = head->x;
		int y = head->y;
	int nextCellState=0;

	int temp[4][8][8];
			memcpy(temp, boardState, sizeof(temp));
	if (head->y == 0) return -1;// ran into wall

	nextCellState = temp[display][y-1][x];

	return nextCellState;
}

int checkOccupiedDown(llist *list, int boardState[][8][8]) {
	struct node *head = *list;
	int display = head->display;
		int x = head->x;
		int y = head->y;

	int temp[4][8][8];
				memcpy(temp, boardState, sizeof(temp));

	int nextCellState=0;

	if (head->y == 7) return -1;// ran into wall

	nextCellState = temp[display][y+1][x];

	return nextCellState;
}

void deleteLastNode(llist *list) {
	struct node *curr = *list;
	while (curr->next->next != NULL) {

		curr = curr->next;
	}
	struct node* lastNode = curr->next;
	curr->next = NULL;
	free(lastNode);
}


void moveRight(llist *list,  int grow) {

	struct node *head = *list;

	int display = head->display;
	int x = head->x;
	int y = head->y;

	if (x == 7) {
		llist_push(list, display+1, y, 0 );
	} else {
		llist_push(list, display, y, x+1 );
	}


		if (grow == 0) deleteLastNode(list);
//		struct node *curr = *list;
//		while (curr->next->next != NULL) {
//
//			curr = curr->next;
//		}
//		struct node* lastNode = curr->next;
//		curr->next = NULL;
//		free(lastNode);






}


void moveLeft(llist *list,  int grow) {

	struct node *head = *list;

	int display = head->display;
	int x = head->x;
	int y = head->y;

		if (head->x == 0) {
			llist_push(list, display-1, y, 7 );
		} else {
			llist_push(list, display, y, x-1 );
		}

		if (grow == 0) deleteLastNode(list);

//		 	struct node* second_last = *list;
//		    while (second_last->next->next != NULL)
//		        second_last = second_last->next;
//
//		    // Delete last node
//		    free (second_last->next);
//
//		    // Change next of second last
//		    second_last->next = NULL;

}

void moveDown(llist *list,  int grow) {

	struct node *head = *list;

	int display = head->display;
		int x = head->x;
		int y = head->y;
	llist_push(list, display, y+1, x );

	if (grow == 0) deleteLastNode(list);
//
//
//	struct node* second_last = *list;
//	while (second_last->next->next != NULL)
//		second_last = second_last->next;
//
//	// Delete last node
//	free (second_last->next);
//
//	// Change next of second last
//	second_last->next = NULL;

}

void moveUp(llist *list,  int grow) {


	struct node *head = *list;

	int display = head->display;
			int x = head->x;
			int y = head->y;

	llist_push(list, display, y-1, x );

	if (grow == 0) deleteLastNode(list);

//	struct node* second_last = *list;
//	while (second_last->next->next != NULL)
//		second_last = second_last->next;
//
//	// Delete last node
//	free (second_last->next);
//
//	// Change next of second last
//	second_last->next = NULL;
}


