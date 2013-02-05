//
//  dataAlign.h
//
//  Created by Jonathan Tompson on 3/14/12.
//  Copyright (c) 2012 NYU. All rights reserved.
//

#ifndef DATAALIGN_HEADER
#define DATAALIGN_HEADER

#ifndef ALIGNMENT
#define ALIGNMENT 16
#endif

#ifdef _WIN32
#define DATA_ALIGN(alignment) __declspec(align(alignment))
#else
#define DATA_ALIGN(alignment) 
#endif

#endif
