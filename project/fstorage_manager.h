/******************************************************************************
File: fstorage_manager.h

This header file contains the macros and function declarations for handling
Flash Storage (fstorage).

******************************************************************************/

#ifndef FSTORAGE_MANAGER_H //Guard statement
#define FSTORAGE_MANAGER_H

/*******************************************************************************
															   PROCEDURES
*******************************************************************************/
void fstorage_init(void);
void fstorage_write_impact( void );
void fstorage_read_impact( void );

#endif /*FSTORAGE_MANAGER_H*/
