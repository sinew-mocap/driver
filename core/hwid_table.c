// SPDX-License-Identifier: MIT
// Copyright (c) 2026-present K. S. Ernest (iFire) Lee
#include "hwid_table.h"
#include <ctype.h>
#include <stdio.h>
#include <string.h>

#define MAX_ENTRIES 256

static SinewHwidEntry g_table[MAX_ENTRIES];
static char g_hwid_buf[MAX_ENTRIES][8];
static char g_joint_buf[MAX_ENTRIES][32];
static int g_table_size = 0;

const SinewHwidEntry *SINEW_HWID_TABLE = g_table;
int SINEW_HWID_TABLE_SIZE = 0;

static char *trim(char *s) {
	while (isspace((unsigned char)*s))
		s++;
	char *e = s + strlen(s);
	while (e > s && isspace((unsigned char)*(e - 1)))
		*(--e) = '\0';
	return s;
}

int sinew_hwid_load(const char *p_path) {
	FILE *f = fopen(p_path, "r");
	if (!f)
		return 0;

	g_table_size = 0;
	int in_hwids = 0;
	char line[256];

	while (fgets(line, sizeof(line), f)) {
		char *s = trim(line);
		if (!*s || *s == ';' || *s == '#')
			continue;

		if (*s == '[') {
			char *end = strchr(s, ']');
			if (end)
				*end = '\0';
			in_hwids = (strcmp(s + 1, "hwids") == 0);
			continue;
		}

		if (!in_hwids)
			continue;

		char *eq = strchr(s, '=');
		if (!eq)
			continue;
		*eq = '\0';
		char *key = trim(s);
		char *val = trim(eq + 1);

		if (!*key || !*val || g_table_size >= MAX_ENTRIES)
			continue;

		strncpy(g_hwid_buf[g_table_size], key, 7);
		g_hwid_buf[g_table_size][7] = '\0';
		strncpy(g_joint_buf[g_table_size], val, 31);
		g_joint_buf[g_table_size][31] = '\0';
		g_table[g_table_size].hwid = g_hwid_buf[g_table_size];
		g_table[g_table_size].joint = g_joint_buf[g_table_size];
		g_table_size++;
	}

	fclose(f);
	SINEW_HWID_TABLE_SIZE = g_table_size;
	return g_table_size;
}

const char *sinew_joint_for_hwid(const char *p_hwid) {
	if (!p_hwid)
		return NULL;
	for (int i = 0; i < SINEW_HWID_TABLE_SIZE; i++) {
		if (strcmp(g_table[i].hwid, p_hwid) == 0)
			return g_table[i].joint;
	}
	return NULL;
}
