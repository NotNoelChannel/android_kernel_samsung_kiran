#ifndef _DEBUGFS_ENERGY_H
#define _DEBUGFS_ENERGY_H

/**
 * @file energy/debugfs_energy.h
 * @author Vyacheslav Cherkashin <v.cherkashin@samsung.com>
 *
 * @section LICENSE
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 * @section COPYRIGHT
 * Copyright (C) Samsung Electronics, 2013
 *
 * @section DESCRIPTION
 * Debugfs for energy
 */


struct dentry;


int init_debugfs_energy(void);
void exit_debugfs_energy(void);

struct dentry *get_energy_dir(void);


#endif /* _DEBUGFS_ENERGY_H */
