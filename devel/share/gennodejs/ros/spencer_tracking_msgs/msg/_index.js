
"use strict";

let TrackingTimingMetrics = require('./TrackingTimingMetrics.js');
let TrackedPerson2d = require('./TrackedPerson2d.js');
let PersonTrajectoryEntry = require('./PersonTrajectoryEntry.js');
let DetectedPersons = require('./DetectedPersons.js');
let TrackedPersons2d = require('./TrackedPersons2d.js');
let PersonTrajectory = require('./PersonTrajectory.js');
let TrackedPerson = require('./TrackedPerson.js');
let TrackedPersons = require('./TrackedPersons.js');
let CompositeDetectedPersons = require('./CompositeDetectedPersons.js');
let CompositeDetectedPerson = require('./CompositeDetectedPerson.js');
let ImmDebugInfos = require('./ImmDebugInfos.js');
let ImmDebugInfo = require('./ImmDebugInfo.js');
let TrackedGroups = require('./TrackedGroups.js');
let DetectedPerson = require('./DetectedPerson.js');
let TrackedGroup = require('./TrackedGroup.js');

module.exports = {
  TrackingTimingMetrics: TrackingTimingMetrics,
  TrackedPerson2d: TrackedPerson2d,
  PersonTrajectoryEntry: PersonTrajectoryEntry,
  DetectedPersons: DetectedPersons,
  TrackedPersons2d: TrackedPersons2d,
  PersonTrajectory: PersonTrajectory,
  TrackedPerson: TrackedPerson,
  TrackedPersons: TrackedPersons,
  CompositeDetectedPersons: CompositeDetectedPersons,
  CompositeDetectedPerson: CompositeDetectedPerson,
  ImmDebugInfos: ImmDebugInfos,
  ImmDebugInfo: ImmDebugInfo,
  TrackedGroups: TrackedGroups,
  DetectedPerson: DetectedPerson,
  TrackedGroup: TrackedGroup,
};
