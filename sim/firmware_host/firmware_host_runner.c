#include "host_stubs.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct SS928HostSnapshot
{
	uint32_t tick_ms;
	int16_t speed_rank;
	float servo_angle;
	float yaw_deg;
	float target_yaw_deg;
	float target_distance_cm;
	float odom_x_cm;
	float odom_y_cm;
	float odom_distance_cm;
	uint8_t run_state;
	uint8_t control_mode;
	uint8_t auto_step;
	int8_t direction;
	int8_t straight_enabled;
	int8_t turn_enabled;
} SS928HostSnapshot_t;

void SS928_HostFirmwareInit(void);
void SS928_HostFirmwareCommand(const char *command);
void SS928_HostFirmwareInject(float yaw_deg, float odom_x_cm, float odom_y_cm, float odom_distance_cm);
void SS928_HostFirmwareTick(uint32_t ms);
void SS928_HostFirmwareSnapshot(SS928HostSnapshot_t *snapshot);

typedef struct PlantState
{
	float yaw_deg;
	float x_cm;
	float y_cm;
	float distance_cm;
} PlantState_t;

static const char *mode_name(uint8_t mode)
{
	static const char *names[] = {"IDLE", "MANUAL", "STRAIGHT", "DISTANCE", "TURN_YAW", "AUTO_ROUTE"};
	return mode < sizeof(names) / sizeof(names[0]) ? names[mode] : "?";
}

static const char *auto_step_name(uint8_t step)
{
	static const char *names[] = {"IDLE", "FORWARD1", "TURN1", "FORWARD2"};
	return step < sizeof(names) / sizeof(names[0]) ? names[step] : "?";
}

static char *trim(char *text)
{
	while(isspace((unsigned char)*text))
	{
		text++;
	}
	char *end = text + strlen(text);
	while(end > text && isspace((unsigned char)end[-1]))
	{
		*--end = '\0';
	}
	return text;
}

static void print_snapshot(const SS928HostSnapshot_t *s)
{
	printf(
		"t=%ums mode=%s auto=%s speed=%d servo=%.2f yaw=%.2f targetYaw=%.2f "
		"dist=%.2f targetDist=%.2f x=%.2f y=%.2f dir=%d straight=%d turn=%d\n",
		(unsigned)s->tick_ms,
		mode_name(s->control_mode),
		auto_step_name(s->auto_step),
		s->speed_rank,
		s->servo_angle,
		s->yaw_deg,
		s->target_yaw_deg,
		s->odom_distance_cm,
		s->target_distance_cm,
		s->odom_x_cm,
		s->odom_y_cm,
		s->direction,
		s->straight_enabled,
		s->turn_enabled
	);
}

static void sync_plant_from_firmware(PlantState_t *plant)
{
	SS928HostSnapshot_t s;
	SS928_HostFirmwareSnapshot(&s);
	plant->yaw_deg = s.yaw_deg;
	plant->x_cm = s.odom_x_cm;
	plant->y_cm = s.odom_y_cm;
	plant->distance_cm = s.odom_distance_cm;
}

static void flush_log(int verbose)
{
	const char *log = SS928_HostLog();
	if(verbose && log[0] != '\0')
	{
		printf("%s", log);
	}
	SS928_HostClearLog();
}

static void plant_drive(uint32_t ms, float yaw_drift_dps, float lateral_drift_cms, PlantState_t *plant)
{
	uint32_t elapsed = 0;
	while(elapsed < ms)
	{
		uint32_t step = ms - elapsed > 10 ? 10 : ms - elapsed;
		SS928HostSnapshot_t s;
		SS928_HostFirmwareSnapshot(&s);

		float dt = (float)step / 1000.0f;
		float speed_level = (float)ABS(s.speed_rank) / (float)SPEEDSTEP;
		float speed_cms = speed_level * (float)RSPEEDSTEP;
		float direction = s.speed_rank >= 0 ? 1.0f : -1.0f;

		if(s.speed_rank != 0)
		{
			plant->distance_cm += speed_cms * dt;
			plant->y_cm += direction * speed_cms * dt;
		}

		if(s.straight_enabled && s.speed_rank != 0)
		{
			plant->yaw_deg += yaw_drift_dps * dt;
			plant->x_cm += lateral_drift_cms * dt;
		}

		if(s.turn_enabled && s.speed_rank != 0)
		{
			float yaw_rate = (s.servo_angle - 90.0f) * 1.5f * direction;
			plant->yaw_deg += yaw_rate * dt;
		}

		while(plant->yaw_deg > 180.0f) plant->yaw_deg -= 360.0f;
		while(plant->yaw_deg < -180.0f) plant->yaw_deg += 360.0f;

		SS928_HostFirmwareInject(plant->yaw_deg, plant->x_cm, plant->y_cm, plant->distance_cm);
		SS928_HostFirmwareTick(step);
		elapsed += step;
	}
}

static int compare_float(float actual, const char *op, float expected)
{
	if(strcmp(op, "==") == 0 || strcmp(op, "=") == 0)
	{
		float delta = actual - expected;
		return delta < 0.001f && delta > -0.001f;
	}
	if(strcmp(op, "!=") == 0) return actual != expected;
	if(strcmp(op, "<") == 0) return actual < expected;
	if(strcmp(op, "<=") == 0) return actual <= expected;
	if(strcmp(op, ">") == 0) return actual > expected;
	if(strcmp(op, ">=") == 0) return actual >= expected;
	return 0;
}

static float snapshot_value(const SS928HostSnapshot_t *s, const char *key)
{
	if(strcmp(key, "tick") == 0) return (float)s->tick_ms;
	if(strcmp(key, "speed") == 0) return (float)s->speed_rank;
	if(strcmp(key, "servo") == 0) return s->servo_angle;
	if(strcmp(key, "yaw") == 0) return s->yaw_deg;
	if(strcmp(key, "target_yaw") == 0) return s->target_yaw_deg;
	if(strcmp(key, "target_distance") == 0) return s->target_distance_cm;
	if(strcmp(key, "distance") == 0) return s->odom_distance_cm;
	if(strcmp(key, "x") == 0) return s->odom_x_cm;
	if(strcmp(key, "y") == 0) return s->odom_y_cm;
	if(strcmp(key, "mode") == 0) return (float)s->control_mode;
	if(strcmp(key, "auto") == 0) return (float)s->auto_step;
	if(strcmp(key, "direction") == 0) return (float)s->direction;
	if(strcmp(key, "straight") == 0) return (float)s->straight_enabled;
	if(strcmp(key, "turn") == 0) return (float)s->turn_enabled;
	fprintf(stderr, "unknown snapshot key: %s\n", key);
	return 0.0f;
}

static int run_line(char *line, PlantState_t *plant, int verbose)
{
	char *text = trim(line);
	if(text[0] == '\0' || text[0] == '#')
	{
		return 0;
	}

	char *op = strtok(text, " \t");
	if(op == NULL)
	{
		return 0;
	}

	if(strcmp(op, "cmd") == 0)
	{
		char *command = trim(strtok(NULL, ""));
		SS928_HostFirmwareCommand(command);
		sync_plant_from_firmware(plant);
		if(verbose) printf("cmd %s\n", command);
		flush_log(verbose);
		return 0;
	}

	if(strcmp(op, "tick") == 0)
	{
		char *ms_text = strtok(NULL, " \t");
		SS928_HostFirmwareTick((uint32_t)strtoul(ms_text, NULL, 10));
		flush_log(verbose);
		return 0;
	}

	if(strcmp(op, "drive") == 0)
	{
		uint32_t ms = (uint32_t)strtoul(strtok(NULL, " \t"), NULL, 10);
		float yaw_drift = 0.0f;
		float lateral_drift = 0.0f;
		char *arg;
		while((arg = strtok(NULL, " \t")) != NULL)
		{
			if(strncmp(arg, "yaw_drift=", 10) == 0)
			{
				yaw_drift = (float)strtod(arg + 10, NULL);
			}
			else if(strncmp(arg, "lateral_drift=", 14) == 0)
			{
				lateral_drift = (float)strtod(arg + 14, NULL);
			}
		}
		plant_drive(ms, yaw_drift, lateral_drift, plant);
		flush_log(verbose);
		return 0;
	}

	if(strcmp(op, "sense") == 0)
	{
		char *arg;
		while((arg = strtok(NULL, " \t")) != NULL)
		{
			if(strncmp(arg, "yaw=", 4) == 0) plant->yaw_deg = (float)strtod(arg + 4, NULL);
			else if(strncmp(arg, "x=", 2) == 0) plant->x_cm = (float)strtod(arg + 2, NULL);
			else if(strncmp(arg, "y=", 2) == 0) plant->y_cm = (float)strtod(arg + 2, NULL);
			else if(strncmp(arg, "distance=", 9) == 0) plant->distance_cm = (float)strtod(arg + 9, NULL);
		}
		SS928_HostFirmwareInject(plant->yaw_deg, plant->x_cm, plant->y_cm, plant->distance_cm);
		return 0;
	}

	if(strcmp(op, "expect") == 0)
	{
		char *key = strtok(NULL, " \t");
		char *cmp = strtok(NULL, " \t");
		char *expected_text = strtok(NULL, " \t");
		SS928HostSnapshot_t s;
		SS928_HostFirmwareSnapshot(&s);
		float actual = snapshot_value(&s, key);
		float expected = (float)strtod(expected_text, NULL);
		if(!compare_float(actual, cmp, expected))
		{
			fprintf(stderr, "FAIL expect %s %s %s, actual %.3f\n", key, cmp, expected_text, actual);
			print_snapshot(&s);
			return 1;
		}
		if(verbose) printf("pass %s %s %s\n", key, cmp, expected_text);
		return 0;
	}

	if(strcmp(op, "print") == 0)
	{
		SS928HostSnapshot_t s;
		SS928_HostFirmwareSnapshot(&s);
		if(verbose)
		{
			print_snapshot(&s);
		}
		return 0;
	}

	fprintf(stderr, "unknown op: %s\n", op);
	return 1;
}

static int run_scenario(FILE *file, int verbose)
{
	char line[512];
	int failures = 0;
	int line_no = 0;
	PlantState_t plant = {0};
	SS928_HostFirmwareInit();

	while(fgets(line, sizeof(line), file) != NULL)
	{
		line_no++;
		if(run_line(line, &plant, verbose) != 0)
		{
			fprintf(stderr, "line %d failed\n", line_no);
			failures++;
		}
	}

	return failures;
}

int main(int argc, char **argv)
{
	const char *scenario = argc > 1 ? argv[1] : "sim/firmware_host/scenarios/basic_control.txt";
	int verbose = argc <= 2 || strcmp(argv[2], "--quiet") != 0;

	FILE *file = fopen(scenario, "r");
	if(file == NULL)
	{
		fprintf(stderr, "failed to open scenario: %s\n", scenario);
		return 2;
	}

	int failures = run_scenario(file, verbose);
	fclose(file);
	if(failures != 0)
	{
		fprintf(stderr, "firmware host simulation failed: %d failure(s)\n", failures);
		return 1;
	}

	printf("firmware host simulation passed\n");
	return 0;
}
