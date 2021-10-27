/*
 * algo.c
 *
 *  Created on: Nov 13, 2020
 *      Author: Łukasz Zelek
 */

#include <algo.h>
#include <cqueue.h>

microMouseState localState;

int8_t rightCorrection[4] = { -16, -1, 16, 1 };
int8_t leftCorrection[4] = { -16, 1, 16, -1 };
uint8_t eastCorrection[2] = { 4, 1 };
uint8_t westCorrection[2] = { 1, 4 };

uint8_t middleOfTheMap[4] = { 0x77, 0x78, 0x87, 0x88 };

uint8_t next[4] = { EAST, SOUTH, WEST, NORTH };
uint8_t previous[4] = { WEST, NORTH, EAST, SOUTH };

void beginFloodingPoint() {
	for (int i = 0; i < MAP_SIZE_N; i++)
		for (int j = 0; j < MAP_SIZE_N; j++)
			localState.flooding[i][j] = 0xff;
}

void fillMaze(Queue *que) {
	while (!empty(que)) {
		int nextToServe = que->front->pos;
		delete(&(que->front->pos), que);
		if (nextToServe / MAP_SIZE_N - 1 > -1) {
			if ((localState.mappedArea[nextToServe / MAP_SIZE_N][nextToServe
					% MAP_SIZE_N] & 8u) != 8)
				if (localState.flooding[nextToServe / MAP_SIZE_N - 1][nextToServe
						% MAP_SIZE_N] == 0xff) {
					set((uint8_t) (nextToServe - MAP_SIZE_N), que);

					localState.flooding[nextToServe / MAP_SIZE_N - 1][nextToServe
							% MAP_SIZE_N] =
							(uint8_t) (localState.flooding[nextToServe
									/ MAP_SIZE_N][nextToServe % MAP_SIZE_N] + 1);

				}
		}
		if (nextToServe % MAP_SIZE_N + 1 < MAP_SIZE_N)
			if ((localState.mappedArea[nextToServe / MAP_SIZE_N][nextToServe
					% MAP_SIZE_N]
					& eastCorrection[localState.mazeTypeRightHanded])
					!= eastCorrection[localState.mazeTypeRightHanded])
				if (localState.flooding[nextToServe / MAP_SIZE_N][nextToServe
						% MAP_SIZE_N + 1] == 0xff) {
					set((uint8_t) (nextToServe + 1), que);

					localState.flooding[nextToServe / MAP_SIZE_N][nextToServe
							% MAP_SIZE_N + 1] =
							(uint8_t) (localState.flooding[nextToServe
									/ MAP_SIZE_N][nextToServe % MAP_SIZE_N] + 1);

				}
		if (nextToServe / MAP_SIZE_N + 1 < MAP_SIZE_N)
			if ((localState.mappedArea[nextToServe / MAP_SIZE_N][nextToServe
					% MAP_SIZE_N] & 2u) != 2)
				if (localState.flooding[nextToServe / MAP_SIZE_N + 1][nextToServe
						% MAP_SIZE_N] == 0xff) {
					set((uint8_t) (nextToServe + MAP_SIZE_N), que);

					localState.flooding[nextToServe / MAP_SIZE_N + 1][nextToServe
							% MAP_SIZE_N] =
							(uint8_t) (localState.flooding[nextToServe
									/ MAP_SIZE_N][nextToServe % MAP_SIZE_N] + 1);

				}
		if (nextToServe % MAP_SIZE_N - 1 > -1)
			if ((localState.mappedArea[nextToServe / MAP_SIZE_N][nextToServe
					% MAP_SIZE_N]
					& westCorrection[localState.mazeTypeRightHanded])
					!= westCorrection[localState.mazeTypeRightHanded])
				if (localState.flooding[nextToServe / MAP_SIZE_N][nextToServe
						% MAP_SIZE_N - 1] == 0xff) {
					set((uint8_t) (nextToServe - 1), que);

					localState.flooding[nextToServe / MAP_SIZE_N][nextToServe
							% MAP_SIZE_N - 1] =
							(uint8_t) (localState.flooding[nextToServe
									/ MAP_SIZE_N][nextToServe % MAP_SIZE_N] + 1);

				}
	}
}

void goForward() {
	switch (localState.rotation) {
	case 0:
		localState.y--;
		break;
	case 1:
		if (localState.mazeTypeRightHanded == 1)
			localState.x--;
		else
			localState.x++;
		break;
	case 2:
		localState.y++;
		break;
	case 3:
		if (localState.mazeTypeRightHanded == 1)
			localState.x++;
		else
			localState.x--;
		break;
	default:
		break;
	}
	localState.actualVisitState = localState.visited[localState.y * MAP_SIZE_N
			+ localState.x];
	localState.visited[localState.y * MAP_SIZE_N + localState.x] = TRUE;
}

void turnLeft() {
	localState.rotation = previous[localState.rotation];
}

void turnRight() {
	localState.rotation = next[localState.rotation];
}

int isWallLeft() {
	if ((localState.actualSensorsState & 4u) == 0)
		return FALSE;
	else
		return TRUE;
}

int isWallRight() {
	if ((localState.actualSensorsState & 1u) == 0)
		return FALSE;
	else
		return TRUE;
}

void fillWalls(int mX, int mY) {
	if (localState.mazeTypeRightHanded == 0) {
		if (mY - 1 > -1)
			if ((localState.mappedArea[mY][mX] & 8u) == 8)
				localState.mappedArea[mY - 1][mX] = (uint8_t) (2u
						| localState.mappedArea[mY - 1][mX]);
		if (mX + 1 < MAP_SIZE_N)
			if ((localState.mappedArea[mY][mX] & 4u) == 4)
				localState.mappedArea[mY][mX + 1] = (uint8_t) (1u
						| localState.mappedArea[mY][mX + 1]);
		if (mY + 1 < MAP_SIZE_N)
			if ((localState.mappedArea[mY][mX] & 2u) == 2)
				localState.mappedArea[mY + 1][mX] = (uint8_t) (8u
						| localState.mappedArea[mY + 1][mX]);
		if (mX - 1 > -1)
			if ((localState.mappedArea[mY][mX] & 1u) == 1)
				localState.mappedArea[mY][mX - 1] = (uint8_t) (4u
						| localState.mappedArea[mY][mX - 1]);
	} else if (localState.mazeTypeRightHanded == 1) {
		if (mY - 1 > -1)
			if ((localState.mappedArea[mY][mX] & 8u) == 8)
				localState.mappedArea[mY - 1][mX] = (uint8_t) (2u
						| localState.mappedArea[mY - 1][mX]);
		if (mX + 1 < MAP_SIZE_N)
			if ((localState.mappedArea[mY][mX] & 1u) == 1)
				localState.mappedArea[mY][mX + 1] = (uint8_t) (4u
						| localState.mappedArea[mY][mX + 1]);
		if (mY + 1 < MAP_SIZE_N)
			if ((localState.mappedArea[mY][mX] & 2u) == 2)
				localState.mappedArea[mY + 1][mX] = (uint8_t) (8u
						| localState.mappedArea[mY + 1][mX]);
		if (mX - 1 > -1)
			if ((localState.mappedArea[mY][mX] & 4u) == 4)
				localState.mappedArea[mY][mX - 1] = (uint8_t) (1u
						| localState.mappedArea[mY][mX - 1]);
	}
}

int correctRotation(int actual, int desired) {
	int result;
	switch (actual - desired) {
	case -3:
		turnLeft();
		result = 1;
		break;
	case -2:
		turnLeft();
		turnLeft();
		result = 0;
		break;
	case -1:
		turnRight();
		result = 3;
		break;
	case 1:
		turnLeft();
		result = 1;
		break;
	case 2:
		turnRight();
		turnRight();
		result = 4;
		break;
	case 3:
		turnRight();
		result = 3;
		break;
	case 0:
		result = 2;
		break;
	default:
		result = 6;
		break;
	}
	return result;
}

int floodFill() {
	int result;
	int decision = 6;
	int instance = -1;
	for (int i = 0; i < 4; i++) {
		if ((localState.mappedArea[localState.y][localState.x] & (8u >> 1u * i))
				== 0) {
			if (localState.mazeTypeRightHanded == 0) {

				int valueOfFloodingLeft = localState.flooding[(localState.y
						* MAP_SIZE_N + localState.x + leftCorrection[i])
						/ MAP_SIZE_N][(localState.y * MAP_SIZE_N + localState.x
						+ leftCorrection[i]) % MAP_SIZE_N];

				if (valueOfFloodingLeft
						== localState.flooding[localState.y][localState.x]) {
					if (instance < 0) {
						instance = 0;
						decision = i;
					} else if (instance == 0)
						if (localState.visited[localState.y * MAP_SIZE_N
								+ localState.x + leftCorrection[i]] == FALSE)
							decision = i;
				} else if (valueOfFloodingLeft
						< localState.flooding[localState.y][localState.x]) {
					if (instance < 1) {
						instance = 1;
						decision = i;
					} else if (instance == 1)
						if (localState.visited[localState.y * MAP_SIZE_N
								+ localState.x + leftCorrection[i]] == FALSE)
							decision = i;
				}
			} else {

				int valueOfFloodingRight = localState.flooding[(localState.y
						* MAP_SIZE_N + localState.x + rightCorrection[i]) /
				MAP_SIZE_N][(localState.y * MAP_SIZE_N + localState.x
						+ rightCorrection[i]) % MAP_SIZE_N];

				if (valueOfFloodingRight
						== localState.flooding[localState.y][localState.x]) {
					if (instance < 0) {
						instance = 0;
						decision = i;
					} else if (instance == 0)
						if (localState.visited[localState.y * MAP_SIZE_N
								+ localState.x + rightCorrection[i]] == FALSE)
							decision = i;
				} else if (valueOfFloodingRight
						< localState.flooding[localState.y][localState.x]) {
					if (instance < 1) {
						instance = 1;
						decision = i;
					} else if (instance == 1)
						if (localState.visited[localState.y * MAP_SIZE_N
								+ localState.x + rightCorrection[i]] == FALSE)
							decision = i;
				}
			}
		}
	}
	if (decision == 6)
		return decision;
	result = correctRotation(localState.rotation, decision);
	goForward();
	return result;
}

int goFastestWay(microMouseState *state) {
	localState = *state;
	if ((localState.y * MAP_SIZE_N + localState.x != middleOfTheMap[0])
			&& (localState.y * MAP_SIZE_N + localState.x != middleOfTheMap[1])
			&& (localState.y * MAP_SIZE_N + localState.x != middleOfTheMap[2])
			&& (localState.y * MAP_SIZE_N + localState.x != middleOfTheMap[3])) {
		int score = floodFill();
		*state = localState;
		return score;
	}
	return -1;
}

int mainMapFunction() {
	if (localState.step == 0) {
		int rawBitMask = localState.actualSensorsState;
		int completeBitMask = rawBitMask + (1u * rawBitMask << 4u);
		localState.mappedArea[localState.y][localState.x] =
				(uint8_t) (localState.mappedArea[localState.y][localState.x]
						| ((1u * completeBitMask
								>> next[next[localState.rotation]]) % MAP_SIZE_N));
		turnLeft();
		turnLeft();
		localState.step++;
		return 0;
	} else if (localState.step == 1) {
		while (1) {
			goForward();
			int rawBitMask = localState.actualSensorsState;
			int completeBitMask = rawBitMask + (1u * rawBitMask << 4u);
			localState.mappedArea[localState.y][localState.x] =
					(uint8_t) (localState.mappedArea[localState.y][localState.x]
							| ((1u * completeBitMask
									>> next[next[localState.rotation]])
									% MAP_SIZE_N));
			int robotPositionY;
			if (!isWallRight()) {
				localState.mazeTypeRightHanded = 1;
				robotPositionY = localState.y;
				while (robotPositionY >= 0) {
					fillWalls(localState.x, robotPositionY);
					robotPositionY--;
				}
				localState.step++;
				break;
			}
			if (!isWallLeft()) {
				localState.mazeTypeRightHanded = 0;
				robotPositionY = localState.y;
				while (robotPositionY >= 0) {
					fillWalls(localState.x, robotPositionY);
					robotPositionY--;
				}
				localState.step++;
				break;
			}
			return 2;
		}
	}
	if (localState.step == 2) {
		if ((localState.y * MAP_SIZE_N + localState.x != middleOfTheMap[0])
				&& (localState.y * MAP_SIZE_N + localState.x
						!= middleOfTheMap[1])
				&& (localState.y * MAP_SIZE_N + localState.x
						!= middleOfTheMap[2])
				&& (localState.y * MAP_SIZE_N + localState.x
						!= middleOfTheMap[3])) {
			beginFloodingPoint();
			for (int i = 0; i < 4; i++)
				localState.flooding[middleOfTheMap[i] / MAP_SIZE_N][middleOfTheMap[i]
						% MAP_SIZE_N] = 0;
			int rawBitMask = localState.actualSensorsState;
			int completeBitMask = rawBitMask + (1u * rawBitMask << 4u);
			localState.mappedArea[localState.y][localState.x] =
					(uint8_t) (localState.mappedArea[localState.y][localState.x]
							| ((1u * completeBitMask
									>> next[next[localState.rotation]])
									% MAP_SIZE_N));
			fillWalls(localState.x, localState.y);
			Queue que;
			initQueue(&que);
			for (int i = 0; i < 4; i++)
				set(middleOfTheMap[i], &que);
			fillMaze(&que);
			int score = floodFill();
			if ((localState.y * MAP_SIZE_N + localState.x == middleOfTheMap[0])
					|| (localState.y * MAP_SIZE_N + localState.x
							== middleOfTheMap[1])
					|| (localState.y * MAP_SIZE_N + localState.x
							== middleOfTheMap[2])
					|| (localState.y * MAP_SIZE_N + localState.x
							== middleOfTheMap[3])) {
				localState.target = localState.y * MAP_SIZE_N + localState.x;
				localState.nextToExplore = -1;
				for (int i = 0; i <= 0xff; i++) {
					if (localState.visited[i] == TRUE) {
						for (int j = 0; j < 4; j++) {
							if ((localState.mappedArea[i / MAP_SIZE_N][i
									% MAP_SIZE_N] & (8u >> 1u * j)) == 0) {
								if (localState.mazeTypeRightHanded == 0) {
									int valueOfFloodingLeft =
											localState.flooding[(i
													+ leftCorrection[j])
													/ MAP_SIZE_N][(i
													+ leftCorrection[j])
													% MAP_SIZE_N];
									if (valueOfFloodingLeft
											< localState.flooding[i / MAP_SIZE_N][i
													% MAP_SIZE_N])
										if (!localState.visited[i
												+ leftCorrection[j]]) {
											localState.nextToExplore = i;
											localState.nodeFound = FALSE, localState.pathReturn =
											FALSE;
											break;
										}
								} else {
									int valueOfFloodingRight =
											localState.flooding[(i
													+ rightCorrection[j]) /
											MAP_SIZE_N][(i + rightCorrection[j])
													% MAP_SIZE_N];
									if (valueOfFloodingRight
											< localState.flooding[i / MAP_SIZE_N][i
													% MAP_SIZE_N])
										if (!localState.visited[i
												+ rightCorrection[j]]) {
											localState.nextToExplore = i;
											localState.nodeFound = FALSE, localState.pathReturn =
											FALSE;
											break;
										}
								}
							}
						}
					}
					if (localState.nextToExplore != -1)
						break;
				}
			}
			return score;
		}
	}
	if (localState.step >= 3 && localState.mapIsFinished == FALSE) {
		if (MAPING_CLUE == 0)
			localState.nextToExplore = -1;
		if (localState.nextToExplore != -1) {
			if (!localState.nodeFound) {
				if (localState.y * MAP_SIZE_N + localState.x
						!= localState.nextToExplore) {
					beginFloodingPoint();
					localState.flooding[localState.nextToExplore / MAP_SIZE_N][localState.nextToExplore
							%
							MAP_SIZE_N] = 0;
					int rawBitMask = localState.actualSensorsState;
					int completeBitMask = rawBitMask + (1u * rawBitMask << 4u);
					localState.mappedArea[localState.y][localState.x] =
							(uint8_t) (localState.mappedArea[localState.y][localState.x]
									| ((1u * completeBitMask
											>> next[next[localState.rotation]])
											% MAP_SIZE_N));
					fillWalls(localState.x, localState.y);
					Queue que;
					initQueue(&que);
					set((uint8_t) localState.nextToExplore, &que);
					fillMaze(&que);
					int score = floodFill();
					if (localState.y * MAP_SIZE_N + localState.x
							== localState.nextToExplore) {
						localState.nodeFound = TRUE;
					}
					return score;
				}
			} else if (!localState.pathReturn) {
				if ((localState.y * MAP_SIZE_N + localState.x
						!= middleOfTheMap[0])
						&& (localState.y * MAP_SIZE_N + localState.x
								!= middleOfTheMap[1])
						&& (localState.y * MAP_SIZE_N + localState.x
								!= middleOfTheMap[2])
						&& (localState.y * MAP_SIZE_N + localState.x
								!= middleOfTheMap[3])) {
					beginFloodingPoint();
					for (int i = 0; i < 4; i++)
						localState.flooding[middleOfTheMap[i] / MAP_SIZE_N][middleOfTheMap[i]
								% MAP_SIZE_N] = 0;
					int rawBitMask = localState.actualSensorsState;
					int completeBitMask = rawBitMask + (1u * rawBitMask << 4u);
					localState.mappedArea[localState.y][localState.x] =
							(uint8_t) (localState.mappedArea[localState.y][localState.x]
									| ((1u * completeBitMask
											>> next[next[localState.rotation]])
											% MAP_SIZE_N));
					fillWalls(localState.x, localState.y);
					Queue que;
					initQueue(&que);
					for (int i = 0; i < 4; i++)
						set(middleOfTheMap[i], &que);
					fillMaze(&que);
					int score = floodFill();
					if (localState.actualVisitState == TRUE) {
						localState.pathReturn = TRUE;
						localState.nextToExplore = -1;
						for (int i = 0; i <= 0xff; i++) {
							if (localState.visited[i] == TRUE) {
								for (int j = 0; j < 4; j++) {
									if ((localState.mappedArea[i / MAP_SIZE_N][i
											% MAP_SIZE_N] & (8u >> 1u * j))
											== 0) {
										if (localState.mazeTypeRightHanded
												== 0) {
											int valueOfFloodingLeft =
													localState.flooding[(i
															+ leftCorrection[j])
															/
															MAP_SIZE_N][(i
															+ leftCorrection[j])
															% MAP_SIZE_N];
											if (valueOfFloodingLeft
													< localState.flooding[i
															/ MAP_SIZE_N][i
															% MAP_SIZE_N])
												if (!localState.visited[i
														+ leftCorrection[j]]) {
													localState.nextToExplore =
															i;
													localState.nodeFound =
													FALSE, localState.pathReturn =
													FALSE;
													break;
												}
										} else {
											int valueOfFloodingRight =
													localState.flooding[(i
															+ rightCorrection[j])
															/
															MAP_SIZE_N][(i
															+ rightCorrection[j])
															% MAP_SIZE_N];
											if (valueOfFloodingRight
													< localState.flooding[i
															/ MAP_SIZE_N][i
															% MAP_SIZE_N])
												if (!localState.visited[i
														+ rightCorrection[j]]) {
													localState.nextToExplore =
															i;
													localState.nodeFound =
													FALSE, localState.pathReturn =
													FALSE;
													break;
												}
										}
									}
								}
							}
							if (localState.nextToExplore != -1)
								break;
						}
					}
					return score;
				}
			}
		} else {
			if (localState.y * MAP_SIZE_N + localState.x != 0) {
				beginFloodingPoint();
				localState.flooding[0][0] = 0;
				int rawBitMask = localState.actualSensorsState;
				int completeBitMask = rawBitMask + (1u * rawBitMask << 4u);
				localState.mappedArea[localState.y][localState.x] =
						(uint8_t) (localState.mappedArea[localState.y][localState.x]
								| ((1u * completeBitMask
										>> next[next[localState.rotation]])
										% MAP_SIZE_N));
				fillWalls(localState.x, localState.y);
				Queue que;
				initQueue(&que);
				set(0, &que);
				fillMaze(&que);
				int score = floodFill();
				if (localState.y * MAP_SIZE_N + localState.x == 0)
					localState.mapIsFinished = TRUE;
				return score;
			}
		}
	}
	if (localState.mapIsFinished == TRUE) {
		beginFloodingPoint();
		Queue que;
		initQueue(&que);
		for (int i = 0; i < 4; i++)
			set(middleOfTheMap[i], &que);
		for (int i = 0; i < 4; i++)
			localState.flooding[middleOfTheMap[i] / MAP_SIZE_N][middleOfTheMap[i]
					% MAP_SIZE_N] = 0;

		for (int i = 0; i <= 0xff; i++) {
			if (localState.visited[i] == TRUE) {
				for (int j = 0; j < 4; j++) {
					if ((localState.mappedArea[i / MAP_SIZE_N][i % MAP_SIZE_N]
							& (8 >> j)) == 0) {
						if (localState.mazeTypeRightHanded == 0) {
							if (!localState.visited[i + leftCorrection[j]]) {
								localState.mappedArea[i / MAP_SIZE_N][i
										% MAP_SIZE_N] =
										((8 >> j)
												| localState.mappedArea[i
														/ MAP_SIZE_N][i
														% MAP_SIZE_N]);
								fillWalls(i % MAP_SIZE_N, i / MAP_SIZE_N);
							}
						} else {
							if (!localState.visited[i + rightCorrection[j]]) {
								localState.mappedArea[i / MAP_SIZE_N][i
										% MAP_SIZE_N] =
										((8 >> j)
												| localState.mappedArea[i
														/ MAP_SIZE_N][i
														% MAP_SIZE_N]);
								fillWalls(i % MAP_SIZE_N, i / MAP_SIZE_N);
							}
						}
					}
				}
			}
		}

		fillMaze(&que);
		return 6;
	}
	return 7;
}

int query(microMouseState *state) {
	localState = *state;
	int score = mainMapFunction();
	if (localState.target != -1)
		localState.step++;
	*state = localState;
	return score;
}

// Merge Function
void merge(int arr[], int l, int m, int r) {
	int i, j, k;
	int n1 = m - l + 1;
	int n2 = r - m;
	int L[n1], R[n2];
	for (i = 0; i < n1; i++)
		L[i] = arr[l + i];
	for (j = 0; j < n2; j++)
		R[j] = arr[m + 1 + j];
	i = 0;
	j = 0;
	k = l;
	while (i < n1 && j < n2) {
		if (L[i] <= R[j]) {
			arr[k] = L[i];
			i++;
		} else {
			arr[k] = R[j];
			j++;
		}
		k++;
	}
	while (i < n1) {
		arr[k] = L[i];
		i++;
		k++;
	}
	while (j < n2) {
		arr[k] = R[j];
		j++;
		k++;
	}
}

/*
 * example of usage
 * mergeSort(arr, 0, arr_size - 1);
 * */
void mergeSort(int arr[], int l, int r) {
	if (l < r) {
		int m = l + (r - l) / 2;
		mergeSort(arr, l, m);
		mergeSort(arr, m + 1, r);
		merge(arr, l, m, r);
	}
}
