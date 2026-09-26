"use strict";

const assert = require("assert");
const crypto = require("crypto");

const studio = require("./app.js");


function pythonPrecisionDocument() {
  return {
    alliance: "blue",
    content_sha256: "073bc91fcc8343130cecfd463e1b5b2024e60f5db8c1deaccef6a3a45f373fae",
    field_version: "2018-power-up-v1",
    plan_id: "hash-precision",
    robots: [
      {
        footprint: { length_m: 0.9, width_m: 0.9 },
        initial_confidence: 0.8,
        max_acceleration_mps2: 1e16,
        max_velocity_mps: 2.0,
        robot_label: "ally-left",
        segments: [
          {
            corridor_radius_m: 0.2,
            earliest_start_us: 0,
            expected_duration_us: 1000000,
            fallback_segment_id: "",
            latest_start_us: 0,
            path: [
              { heading: 0.7853981633974483, time_us: 0, x: 1.0, y: 1.0 },
              { heading: 0.7853981633974483, time_us: 500000, x: 1.5, y: 1.0 },
              { heading: 0.7853981633974483, time_us: 1000000, x: 2.0, y: 1.0 },
            ],
            segment_id: "cross-line-1",
            target_id: "auto-line",
            task: "CROSS_LINE",
          },
        ],
        start_pose: { heading: 0.7853981633974483, x: 1.0, y: 1.0 },
        team_number: 254,
      },
    ],
    schema_version: 1,
  };
}


function hash(value) {
  return crypto.createHash("sha256").update(value, "utf8").digest("hex");
}


function testPreservesPythonFloatPrecisionAndExponentFormatting() {
  const document = pythonPrecisionDocument();
  const imported = studio.importCanonicalDocument(document);
  const canonical = studio.canonicalStringify(studio.canonicalPayload(imported));

  assert.strictEqual(hash(canonical), document.content_sha256);
  assert.match(canonical, /0\.7853981633974483/);
  assert.match(canonical, /1e\+16/);
}


function testDetectsCollisionBetweenHundredMillisecondSamples() {
  const robot = (teamNumber, points) => ({
    teamNumber,
    label: `ally-${teamNumber}`,
    footprintLength: 0.05,
    footprintWidth: 0.05,
    maxVelocity: 100,
    maxAcceleration: 1000,
    initialConfidence: 0.8,
    segments: [
      {
        id: `route-${teamNumber}`,
        task: "CROSS_LINE",
        targetId: "auto-line",
        startSeconds: 0,
        durationSeconds: 0.1,
        corridorRadius: 0,
        points,
      },
    ],
  });
  const state = {
    robots: [
      robot(254, [
        { timeUs: 0, x: 0, y: 0, heading: 0 },
        { timeUs: 100000, x: 1, y: 0, heading: 0 },
      ]),
      robot(1678, [
        { timeUs: 0, x: 0.5, y: 1, heading: 0 },
        { timeUs: 100000, x: 0.5, y: -1, heading: 0 },
      ]),
    ],
  };

  const conflicts = studio.analyzeConflicts(state);

  assert.strictEqual(conflicts.length, 1);
  assert.ok(conflicts[0].startUs < 50000);
  assert.ok(conflicts[0].endUs > 50000);
}


function testMirrorsWireBoundsAndSegmentContinuityDiagnostics() {
  const state = studio.importCanonicalDocument(pythonPrecisionDocument());
  state.robots[0].teamNumber = 2147483648;
  state.robots[0].segments.push({
    id: "teleport",
    task: "WAIT",
    targetId: "unsafe",
    startSeconds: 1,
    durationSeconds: 1,
    corridorRadius: 0.2,
    points: [
      { timeUs: 1000000, x: 10, y: 1, heading: 0 },
      { timeUs: 2000000, x: 10, y: 1, heading: 0 },
    ],
  });

  const errors = studio.validateState(state);

  assert.ok(errors.some((error) => error.includes("int32")));
  assert.ok(errors.some((error) => error.includes("kopuk")));
}


function testRoundTripsStartWindowsFallbacksAndIndependentStartPose() {
  const document = {
    schema_version: 1,
    field_version: "2018-power-up-v1",
    alliance: "blue",
    plan_id: "qualification-12-blue",
    content_sha256: "ff71ec2e074aa060a3565eeea33dbff4cbf98d9c28cfb7aa48b8c1c3542e7a09",
    robots: [{
      team_number: 254,
      robot_label: "ally-left",
      start_pose: { x: 0.5, y: 0.5, heading: 0.25 },
      footprint: { length_m: 0.9, width_m: 0.9 },
      max_velocity_mps: 2.0,
      max_acceleration_mps2: 4.0,
      initial_confidence: 0.85,
      segments: [
        {
          segment_id: "cross-line",
          task: "CROSS_LINE",
          target_id: "auto-line",
          earliest_start_us: 0,
          latest_start_us: 100000,
          expected_duration_us: 1000000,
          corridor_radius_m: 0.2,
          fallback_segment_id: "wait-safe",
          path: [
            { time_us: 50000, x: 1.0, y: 1.0, heading: 0.0 },
            { time_us: 1050000, x: 2.0, y: 1.0, heading: 0.0 },
          ],
        },
        {
          segment_id: "wait-safe",
          task: "WAIT",
          target_id: "safe-zone",
          earliest_start_us: 1050000,
          latest_start_us: 1050000,
          expected_duration_us: 1000000,
          corridor_radius_m: 0.2,
          fallback_segment_id: "",
          path: [
            { time_us: 1050000, x: 2.0, y: 1.0, heading: 0.0 },
            { time_us: 2050000, x: 2.0, y: 1.0, heading: 0.0 },
          ],
        },
      ],
    }],
  };

  const imported = studio.importCanonicalDocument(document);
  const canonical = studio.canonicalStringify(studio.canonicalPayload(imported));

  assert.strictEqual(hash(canonical), document.content_sha256);
}


function testRejectsMissingFallbackBeforeExport() {
  const state = studio.importCanonicalDocument(pythonPrecisionDocument());
  state.robots[0].segments[0].fallbackSegmentId = "missing-segment";

  const errors = studio.validateState(state);

  assert.ok(errors.some((error) => error.includes("fallback")));
}


testPreservesPythonFloatPrecisionAndExponentFormatting();
testDetectsCollisionBetweenHundredMillisecondSamples();
testMirrorsWireBoundsAndSegmentContinuityDiagnostics();
testRoundTripsStartWindowsFallbacksAndIndependentStartPose();
testRejectsMissingFallbackBeforeExport();
console.log("Strategy Studio tests passed");
