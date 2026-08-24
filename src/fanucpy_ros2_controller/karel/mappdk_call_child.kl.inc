----------------------------------------------------
-- Copyright 2026 ureed
-- SPDX-License-Identifier: Apache-2.0
--
-- Modified controller routine for the Apache-2.0
-- fanucpy MAPPDK server originally developed by
-- Agajan Torayev and contributors.
--
-- Replace the complete MAPPDKCALL routine in
-- mappdk_cmd.kl with this routine. Also add:
--     %ENVIRONMENT MULTI
-- to mappdk_server.kl before translating it.
----------------------------------------------------

ROUTINE MAPPDKCALL(cmd: STRING) : STRING
----------------------------------------------------
-- Run an external TP program as an isolated child
-- task, wait for it to finish, and keep the MAPPDK
-- socket server in its parent task.
----------------------------------------------------
VAR
    resp:           STRING[254]
    prg_name:       STRING[32]
    task_info:      STRING[32]
    status_text:    STRING[12]
    name_len:       INTEGER
    task_no:        INTEGER
    task_state:     INTEGER
    status:         INTEGER
BEGIN
    -- Command format: mappdkcall:PROG_NAME
    name_len = STR_LEN(cmd) - 11
    IF (name_len < 1) OR (name_len > 32) THEN
        RETURN('1:invalid-program-name-length')
    ENDIF

    prg_name = SUB_STR(cmd, 12, name_len)
    task_no = 0
    status = 0

    -- MAPPDK_SERVER uses %NOLOCKGROUP. The TP child obtains its own
    -- configured motion group; no group is transferred by the parent.
    RUN_TASK(prg_name, 0, FALSE, FALSE, 0, status)
    IF status <> 0 THEN
        CNV_INT_STR(status, 10, 0, status_text)
        RETURN('1:run-task-failed-' + status_text)
    ENDIF

    -- A completed FANUC task is reported as PG_ABORTED. Keep the socket
    -- command blocked until then, preserving the existing fanucpy API.
    REPEAT
        task_state = PG_RUNACCEPT
        task_info = ''
        GET_TSK_INFO(prg_name, task_no, TSK_STATUS,
                     task_state, task_info, status)
        IF status <> 0 THEN
            CNV_INT_STR(status, 10, 0, status_text)
            RETURN('1:task-status-failed-' + status_text)
        ENDIF

        IF task_state <> PG_ABORTED THEN
            DELAY(100)
        ENDIF
    UNTIL task_state = PG_ABORTED

    resp = '0:success'
    RETURN(resp)
END MAPPDKCALL
