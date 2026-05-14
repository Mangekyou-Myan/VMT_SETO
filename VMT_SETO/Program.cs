using System.Diagnostics;
using System.Globalization;
using System.Numerics;
using System.Text;
using System.Windows.Forms;
using Valve.VR;

namespace VmtSeto
{
    enum TrackingSpaceMode
    {
        Raw,
        Room
    }

    enum CalibrationControlMode
    {
        Gui,
        ToggleKey,
        HoldKey,
        Off
    }

    class BodyPartConfig
    {
        public string Name = string.Empty;
        public Vector3 PositionOffset;
        public Vector3 RotationOffset;
        public float RollOffset;
        public bool DebugOutput;
        public string SerialNumber = string.Empty;
        public int VmtId;
        public uint DeviceIndex = OpenVR.k_unTrackedDeviceIndexInvalid;
        public long NextDeviceSearchTick;
        public bool LastEnableSent;
        public bool HasEverHadSourcePose;
        public Vector3 LatestLivePosition;
        public Quaternion LatestLiveRotation = Quaternion.Identity;
        public bool HasLatestLivePose;
        public Vector3 LastPosition;
        public Vector3 Velocity;
        public OneEuroVector3Filter PredictionOffsetFilter = new();
        public long LastPositionTick;
        public bool HasLastPosition;
    }

    class PresetChoice
    {
        public string Name = string.Empty;
        public string ConfigPath = string.Empty;

        public override string ToString()
        {
            return Name;
        }
    }

    class BodyConfigChoice
    {
        public BodyPartConfig Config;

        public BodyConfigChoice(BodyPartConfig config)
        {
            Config = config;
        }

        public override string ToString()
        {
            return $"{Config.Name} / VMT {Config.VmtId}";
        }
    }

    class AppSettings
    {
        public int Fps = 360;
        public TrackingSpaceMode TrackingSpace = TrackingSpaceMode.Raw;
        public bool YawOnlyPositionOffset = true;
        public Vector3 HmdViewpointOffset = Vector3.Zero;
        public string TargetHost = "127.0.0.1";
        public int TargetPort = 39570;
        public float TimeOffset = 0f;
        public bool CreateTrackersWithoutSource = true;
        public bool DisableTrackerWhenSourceLost = true;
        public bool ControllerUnlockCalibration = true;
        public bool ControllerUnlockWithTrigger = true;
        public bool ControllerUnlockWithGrip = true;
        public float PosePredictionSeconds = 0.005f;
        public float PositionPredictionSeconds = 0.025f;
        public float PositionPredictionStrength = 0.4f;
        public float PositionPredictionMaxSpeed = 8f;
        public float PositionVelocitySampleSeconds = 0.012f;
        public float PositionVelocityBlend = 0.25f;
        public float PositionVelocityDeadzone = 0.03f;
        public bool PositionOneEuroEnabled = true;
        public float PositionOneEuroMinCutoff = 2.5f;
        public float PositionOneEuroBeta = 0.2f;
        public float PositionOneEuroDerivativeCutoff = 1.0f;
        public bool BusyWait = true;
        public CalibrationControlMode CalibrationControl = CalibrationControlMode.Gui;
        public int CalibrationToggleKey = 0x77; // F8
        public int CalibrationHoldKey = 0x20; // Space
        public bool CalibrationYawRotation = true;
        public bool CalibrationLockAnchorPosition = false;
        public string SelectedConfigPath = string.Empty;
    }

    class OneEuroVector3Filter
    {
        readonly OneEuroFloatFilter x = new();
        readonly OneEuroFloatFilter y = new();
        readonly OneEuroFloatFilter z = new();

        public Vector3 Filter(Vector3 value, double timestampSeconds, float minCutoff, float beta, float derivativeCutoff)
        {
            return new Vector3(
                x.Filter(value.X, timestampSeconds, minCutoff, beta, derivativeCutoff),
                y.Filter(value.Y, timestampSeconds, minCutoff, beta, derivativeCutoff),
                z.Filter(value.Z, timestampSeconds, minCutoff, beta, derivativeCutoff));
        }

        public void Reset()
        {
            x.Reset();
            y.Reset();
            z.Reset();
        }
    }

    class OneEuroFloatFilter
    {
        bool initialized;
        float lastRawValue;
        float filteredValue;
        float filteredDerivative;
        double lastTimestampSeconds;

        public float Filter(float value, double timestampSeconds, float minCutoff, float beta, float derivativeCutoff)
        {
            if (!initialized)
            {
                initialized = true;
                lastRawValue = value;
                filteredValue = value;
                filteredDerivative = 0f;
                lastTimestampSeconds = timestampSeconds;
                return value;
            }

            float deltaSeconds = (float)(timestampSeconds - lastTimestampSeconds);
            if (deltaSeconds <= 0f)
            {
                return filteredValue;
            }

            if (deltaSeconds > 0.25f)
            {
                Reset();
                return Filter(value, timestampSeconds, minCutoff, beta, derivativeCutoff);
            }

            float rawDerivative = (value - lastRawValue) / deltaSeconds;
            float derivativeAlpha = FilterAlpha(deltaSeconds, derivativeCutoff);
            filteredDerivative = LowPass(filteredDerivative, rawDerivative, derivativeAlpha);

            float cutoff = minCutoff + beta * MathF.Abs(filteredDerivative);
            float valueAlpha = FilterAlpha(deltaSeconds, cutoff);
            filteredValue = LowPass(filteredValue, value, valueAlpha);

            lastRawValue = value;
            lastTimestampSeconds = timestampSeconds;
            return filteredValue;
        }

        public void Reset()
        {
            initialized = false;
            lastRawValue = 0f;
            filteredValue = 0f;
            filteredDerivative = 0f;
            lastTimestampSeconds = 0d;
        }

        static float FilterAlpha(float deltaSeconds, float cutoff)
        {
            float safeCutoff = MathF.Max(0.001f, cutoff);
            float tau = 1f / (2f * MathF.PI * safeCutoff);
            return 1f / (1f + tau / deltaSeconds);
        }

        static float LowPass(float previousValue, float value, float alpha)
        {
            return previousValue + alpha * (value - previousValue);
        }
    }

    class Program
    {
        const int EscapeKey = 0x1B;
        const string TypeTag = ",iiffffffff";

        static readonly List<BodyPartConfig> Configs = new();
        static readonly System.Net.Sockets.UdpClient Sender = new();
        static readonly byte[] OscBuffer = new byte[256];
        static readonly object PoseStateLock = new();
        static readonly object TriggerCaptureLock = new();

        static AppSettings settings = new();
        static string oscAddress = "/VMT/Raw/Unity";
        static ETrackingUniverseOrigin trackingOrigin = ETrackingUniverseOrigin.TrackingUniverseRawAndUncalibrated;
        static Vector3 latestHmdPosition;
        static Quaternion latestHmdRotation = Quaternion.Identity;
        static Quaternion latestCalibrationBaseRotation = Quaternion.Identity;
        static Vector3 latestCalibrationReferencePosition;
        static bool hasLatestCalibrationReferencePosition;
        static Vector3 calibrationAnchorPosition;
        static bool calibrationAnchorPositionValid;
        static bool lastCalibrationActiveState;
        static bool hasLatestHmdPose;
        static volatile bool calibrationActive;
        static volatile bool exitRequested;
        static bool toggleKeyWasDown;
        static readonly List<BodyPartConfig> TriggerCaptureTargets = new();
        static string triggerCaptureStatus = string.Empty;
        static bool triggerCaptureArmed;
        static bool triggerCaptureIgnoreUntilReleased;
        static bool controllerTriggerWasDown;
        static bool controllerGripWasDown;
        static int lastStatsLineCount;
        static string? pendingConfigReloadPath;
        static Thread? controlUiThread;
        static CalibrationForm? calibrationForm;

        [STAThread]
        static void Main(string[] args)
        {
            bool timerPeriodSet = NativeMethods.TimeBeginPeriod(1) == 0;
            try
            {
                MainCore(args);
            }
            finally
            {
                if (timerPeriodSet)
                {
                    NativeMethods.TimeEndPeriod(1);
                }
            }
        }

        static void MainCore(string[] args)
        {
            Console.WriteLine("=== VMT SETO start ===");
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);

            string? configPath = ResolveConfigPath(args);
            if (configPath == null)
            {
                return;
            }

            if (!LoadConfig(configPath))
            {
                return;
            }
            ApplyRuntimeHints();
            StartControlUi();

            Sender.Connect(settings.TargetHost, settings.TargetPort);
            SendCreateAllAtOrigin(force: true);

            var error = EVRInitError.None;
            var vrSystem = OpenVR.Init(ref error, EVRApplicationType.VRApplication_Background);
            if (error != EVRInitError.None)
            {
                Console.WriteLine($"SteamVR init failed: {error}");
                ShutdownControlUi();
                Sender.Dispose();
                return;
            }

            try
            {
                RunLoop(vrSystem);
            }
            finally
            {
                exitRequested = true;
                SendDisableAll(force: true);
                ShutdownControlUi();
                Sender.Dispose();
                OpenVR.Shutdown();
            }
        }

        static void RunLoop(CVRSystem vrSystem)
        {
            var poses = new TrackedDevicePose_t[OpenVR.k_unMaxTrackedDeviceCount];
            double ticksPerFrame = settings.Fps > 0 ? Stopwatch.Frequency / (double)settings.Fps : 0d;
            double nextFrameTick = Stopwatch.GetTimestamp();
            long lastStatsTick = Stopwatch.GetTimestamp();
            int frameCount = 0;
            int sentCount = 0;
            int measuredFrameCount = 0;
            long poseTicksTotal = 0;
            long waitTicksTotal = 0;
            Vector3 lastHmdPos = Vector3.Zero;
            bool lastCalibrating = false;

            Console.WriteLine($"Config: {settings.SelectedConfigPath}");
            Console.WriteLine($"OSC: {oscAddress} -> {settings.TargetHost}:{settings.TargetPort}");
            Console.WriteLine($"TrackingSpace: {settings.TrackingSpace}, FPS: {(settings.Fps > 0 ? settings.Fps.ToString(CultureInfo.InvariantCulture) : "Unlimited")}, PredictionMs: {settings.PosePredictionSeconds * 1000f:F1}, BusyWait: {settings.BusyWait}, YawOnlyPositionOffset: {settings.YawOnlyPositionOffset}, CreateWithoutSource: {settings.CreateTrackersWithoutSource}, DisableWhenLost: {settings.DisableTrackerWhenSourceLost}");
            Console.WriteLine($"HmdViewpointOffset: {settings.HmdViewpointOffset.X:F4}, {settings.HmdViewpointOffset.Y:F4}, {settings.HmdViewpointOffset.Z:F4}");
            Console.WriteLine($"PositionPrediction: {settings.PositionPredictionStrength * 100f:F0}% / {EffectivePositionPredictionSeconds() * 1000f:F1}ms, VelocityWindow: {settings.PositionVelocitySampleSeconds * 1000f:F1}ms, VelocityBlend: {settings.PositionVelocityBlend:F2}");
            Console.WriteLine($"1Euro: {(settings.PositionOneEuroEnabled ? "On" : "Off")}, MinCutoff: {settings.PositionOneEuroMinCutoff:F2}, Beta: {settings.PositionOneEuroBeta:F2}, DerivativeCutoff: {settings.PositionOneEuroDerivativeCutoff:F2}");
            Console.WriteLine($"Calibration: {settings.CalibrationControl}, ToggleKey: {KeyName(settings.CalibrationToggleKey)}, HoldKey: {KeyName(settings.CalibrationHoldKey)}, YawRotation: {settings.CalibrationYawRotation}, LockAnchor: {settings.CalibrationLockAnchorPosition}, ControllerUnlock: {settings.ControllerUnlockCalibration}");
            Console.WriteLine("Esc: exit");

            while (!exitRequested && !NativeMethods.IsKeyDown(EscapeKey))
            {
                if (TryConsumePendingConfigReload(out string reloadConfigPath))
                {
                    if (ApplyConfigReload(reloadConfigPath))
                    {
                        ticksPerFrame = settings.Fps > 0 ? Stopwatch.Frequency / (double)settings.Fps : 0d;
                        nextFrameTick = Stopwatch.GetTimestamp();
                    }
                }

                long now = Stopwatch.GetTimestamp();
                UpdateCalibrationInputState();
                long poseStartTick = Stopwatch.GetTimestamp();
                vrSystem.GetDeviceToAbsoluteTrackingPose(trackingOrigin, settings.PosePredictionSeconds, poses);
                long poseEndTick = Stopwatch.GetTimestamp();
                poseTicksTotal += poseEndTick - poseStartTick;
                measuredFrameCount++;

                var hmd = poses[0];
                bool hmdValid = hmd.bDeviceIsConnected && hmd.bPoseIsValid;
                if (hmdValid)
                {
                    Vector3 hmdPos = GetUnityPosition(hmd.mDeviceToAbsoluteTracking);
                    Quaternion hmdRot = GetUnityRotation(hmd.mDeviceToAbsoluteTracking);
                    Quaternion offsetBaseRot = GetCalibrationBaseRotation(hmd.mDeviceToAbsoluteTracking, hmdRot);
                    bool isCalibrating = calibrationActive;
                    Vector3 calibrationReferencePos = UpdateCalibrationReferencePosition(hmdPos, hmdRot, offsetBaseRot, isCalibrating);

                    lastHmdPos = hmdPos;
                    lastCalibrating = isCalibrating;

                    foreach (var conf in Configs)
                    {
                        EnsureDeviceIndex(vrSystem, poses, conf, now);

                        Vector3 sendPos;
                        Quaternion sendRot;
                        bool hasTrackerPose = conf.DeviceIndex != OpenVR.k_unTrackedDeviceIndexInvalid
                            && poses[conf.DeviceIndex].bPoseIsValid;
                        Vector3 livePos = Vector3.Zero;
                        Quaternion liveRot = Quaternion.Identity;

                        if (hasTrackerPose)
                        {
                            conf.HasEverHadSourcePose = true;
                            var pose = poses[conf.DeviceIndex].mDeviceToAbsoluteTracking;
                            livePos = GetUnityPosition(pose);
                            liveRot = GetUnityRotation(pose);

                            lock (PoseStateLock)
                            {
                                conf.LatestLivePosition = livePos;
                                conf.LatestLiveRotation = liveRot;
                                conf.HasLatestLivePose = true;
                            }
                        }
                        else
                        {
                            lock (PoseStateLock)
                            {
                                conf.HasLatestLivePose = false;
                            }
                        }

                        if (isCalibrating)
                        {
                            sendPos = calibrationReferencePos + Vector3.Transform(conf.PositionOffset, offsetBaseRot);
                            Quaternion offsetRot = CreateCalibrationOffsetRotation(conf);
                            sendRot = settings.CalibrationYawRotation
                                ? Quaternion.Normalize(offsetBaseRot * offsetRot)
                                : offsetRot;
                            ResetPositionPrediction(conf);
                        }
                        else if (hasTrackerPose)
                        {
                            sendPos = ApplyPositionPrediction(conf, livePos, poseEndTick);
                            sendRot = liveRot;
                        }
                        else if (settings.CreateTrackersWithoutSource
                            && (!settings.DisableTrackerWhenSourceLost || !conf.HasEverHadSourcePose))
                        {
                            sendPos = calibrationReferencePos + Vector3.Transform(conf.PositionOffset, offsetBaseRot);
                            Quaternion offsetRot = CreateCalibrationOffsetRotation(conf);
                            sendRot = settings.CalibrationYawRotation
                                ? Quaternion.Normalize(offsetBaseRot * offsetRot)
                                : offsetRot;
                            ResetPositionPrediction(conf);
                        }
                        else
                        {
                            ResetPositionPrediction(conf);
                            SendPose(conf, enable: false, Vector3.Zero, Quaternion.Identity);
                            continue;
                        }

                        SendPose(conf, enable: true, sendPos, sendRot);
                        sentCount++;
                    }
                }
                else
                {
                    lock (PoseStateLock)
                    {
                        hasLatestHmdPose = false;
                        hasLatestCalibrationReferencePosition = false;
                        latestCalibrationBaseRotation = Quaternion.Identity;
                        calibrationAnchorPositionValid = false;
                        lastCalibrationActiveState = false;
                    }

                    if (settings.CreateTrackersWithoutSource)
                    {
                        SendCreateAllAtOrigin(force: false);
                    }
                    else
                    {
                        SendDisableAll(force: false);
                    }
                }

                UpdateTriggerCapture(vrSystem);

                frameCount++;
                long statsNow = Stopwatch.GetTimestamp();
                if (statsNow - lastStatsTick >= Stopwatch.Frequency)
                {
                    double elapsed = (statsNow - lastStatsTick) / (double)Stopwatch.Frequency;
                    double actualFps = frameCount / elapsed;
                    double avgPoseMs = measuredFrameCount > 0 ? poseTicksTotal * 1000.0 / Stopwatch.Frequency / measuredFrameCount : 0.0;
                    double avgWaitMs = measuredFrameCount > 0 ? waitTicksTotal * 1000.0 / Stopwatch.Frequency / measuredFrameCount : 0.0;
                    PrintStats(actualFps, sentCount, avgPoseMs, avgWaitMs, lastHmdPos, hmdValid, lastCalibrating);
                    frameCount = 0;
                    sentCount = 0;
                    measuredFrameCount = 0;
                    poseTicksTotal = 0;
                    waitTicksTotal = 0;
                    lastStatsTick = statsNow;
                }

                if (ticksPerFrame > 0d)
                {
                    nextFrameTick += ticksPerFrame;
                    long waitStartTick = Stopwatch.GetTimestamp();
                    WaitUntil((long)nextFrameTick);
                    long waitEndTick = Stopwatch.GetTimestamp();
                    waitTicksTotal += waitEndTick - waitStartTick;

                    long afterWait = waitEndTick;
                    if (afterWait - nextFrameTick > ticksPerFrame)
                    {
                        nextFrameTick = afterWait;
                    }
                }
            }
        }

        static bool LoadConfig(string configPath)
        {
            if (!File.Exists(configPath))
            {
                Console.WriteLine($"Error: config.txt not found. ({configPath})");
                return false;
            }

            settings = new AppSettings { SelectedConfigPath = configPath };
            Configs.Clear();

            foreach (var rawLine in File.ReadAllLines(configPath))
            {
                string line = StripComment(rawLine).Trim();
                if (line.Length == 0) continue;

                int settingSeparator = line.IndexOf('=');
                if (settingSeparator > 0)
                {
                    ParseSetting(line[..settingSeparator].Trim(), line[(settingSeparator + 1)..].Trim());
                    continue;
                }

                ParseBodyPartConfig(line);
            }

            settings.SelectedConfigPath = configPath;
            ApplySettings();
            Console.WriteLine($"Loaded {Configs.Count} body config(s). ({configPath})");
            return true;
        }

        static string? ResolveConfigPath(string[] args)
        {
            string? argConfigPath = TryGetArgumentValue(args, "--config");
            if (!string.IsNullOrWhiteSpace(argConfigPath))
            {
                return ResolvePath(AppContext.BaseDirectory, argConfigPath);
            }

            string? argPresetName = TryGetArgumentValue(args, "--preset");
            if (!string.IsNullOrWhiteSpace(argPresetName))
            {
                return ResolvePresetPath(argPresetName);
            }

            if (args.Length == 1 && !args[0].StartsWith("-", StringComparison.Ordinal))
            {
                return ResolvePath(AppContext.BaseDirectory, args[0]);
            }

            string launcherConfigPath = ResolveDefaultLauncherConfigPath();
            string? selectedConfig = TryReadSelectorValue(launcherConfigPath, "config");
            if (!string.IsNullOrWhiteSpace(selectedConfig))
            {
                return ResolvePath(Path.GetDirectoryName(launcherConfigPath) ?? AppContext.BaseDirectory, selectedConfig);
            }

            string? selectedPreset = TryReadSelectorValue(launcherConfigPath, "preset");
            if (!string.IsNullOrWhiteSpace(selectedPreset))
            {
                return ResolvePresetPath(selectedPreset);
            }

            return launcherConfigPath;
        }

        static List<PresetChoice> FindPresetChoices()
        {
            var choices = new List<PresetChoice>();
            var seenPaths = new HashSet<string>(StringComparer.OrdinalIgnoreCase);

            foreach (string presetsRoot in GetPresetRoots())
            {
                if (!Directory.Exists(presetsRoot)) continue;

                foreach (string presetDirectory in Directory.GetDirectories(presetsRoot))
                {
                    string configPath = Path.Combine(presetDirectory, "config.txt");
                    if (!File.Exists(configPath)) continue;

                    string fullPath = Path.GetFullPath(configPath);
                    if (!seenPaths.Add(fullPath)) continue;

                    choices.Add(new PresetChoice
                    {
                        Name = Path.GetFileName(presetDirectory),
                        ConfigPath = fullPath
                    });
                }
            }

            return choices
                .OrderBy(choice => choice.Name, StringComparer.OrdinalIgnoreCase)
                .ToList();
        }

        static List<BodyPartConfig> GetBodyConfigs()
        {
            return Configs.ToList();
        }

        static string CaptureCurrentPose(BodyPartConfig config)
        {
            return CaptureCurrentPoses(new[] { config });
        }

        static string CaptureCurrentPoses(IReadOnlyList<BodyPartConfig> configs)
        {
            var uniqueConfigs = UniqueConfigs(configs);
            if (uniqueConfigs.Count == 0)
            {
                return "Capture failed: no trackers selected.";
            }

            var capturedNames = new List<string>();
            var skippedNames = new List<string>();

            lock (PoseStateLock)
            {
                if (!hasLatestHmdPose)
                {
                    return "Capture failed: HMD pose is not valid.";
                }

                Quaternion baseRot = latestCalibrationBaseRotation;
                Quaternion inverseBaseRot = Quaternion.Inverse(baseRot);
                Vector3 referencePosition = hasLatestCalibrationReferencePosition
                    ? latestCalibrationReferencePosition
                    : latestHmdPosition;

                foreach (var config in uniqueConfigs)
                {
                    if (!config.HasLatestLivePose)
                    {
                        skippedNames.Add(config.Name);
                        continue;
                    }

                    CaptureCurrentPoseLocked(config, inverseBaseRot, referencePosition);
                    capturedNames.Add(config.Name);
                }
            }

            if (capturedNames.Count == 0)
            {
                return $"Capture failed: tracker pose is not valid. ({FormatTrackerNames(skippedNames)})";
            }

            if (skippedNames.Count > 0)
            {
                return $"Captured: {capturedNames.Count}, skipped: {FormatTrackerNames(skippedNames)}";
            }

            return capturedNames.Count == 1
                ? $"Captured: {capturedNames[0]}"
                : $"Captured: {capturedNames.Count} trackers";
        }

        static void CaptureCurrentPoseLocked(BodyPartConfig config, Quaternion inverseBaseRot, Vector3 referencePosition)
        {
            Vector3 positionOffset = Vector3.Transform(config.LatestLivePosition - referencePosition, inverseBaseRot);

            Quaternion rotationOffset = settings.CalibrationYawRotation
                ? Quaternion.Normalize(inverseBaseRot * config.LatestLiveRotation)
                : Quaternion.Normalize(config.LatestLiveRotation);

            config.PositionOffset = positionOffset;
            config.RotationOffset = GetUnityEulerFromQuaternion(rotationOffset);
            config.RollOffset = 0f;
        }

        static string ArmTriggerCapture(IReadOnlyList<BodyPartConfig> configs)
        {
            var uniqueConfigs = UniqueConfigs(configs);
            if (uniqueConfigs.Count == 0)
            {
                return "Arm failed: no trackers selected.";
            }

            lock (TriggerCaptureLock)
            {
                TriggerCaptureTargets.Clear();
                TriggerCaptureTargets.AddRange(uniqueConfigs);
                triggerCaptureArmed = true;
                triggerCaptureIgnoreUntilReleased = true;
                triggerCaptureStatus = uniqueConfigs.Count == 1
                    ? $"Waiting trigger: {uniqueConfigs[0].Name}"
                    : $"Waiting trigger: {uniqueConfigs.Count} trackers";
                return triggerCaptureStatus;
            }
        }

        static List<BodyPartConfig> UniqueConfigs(IEnumerable<BodyPartConfig> configs)
        {
            var uniqueConfigs = new List<BodyPartConfig>();
            var seenIds = new HashSet<int>();

            foreach (var config in configs)
            {
                if (seenIds.Add(config.VmtId))
                {
                    uniqueConfigs.Add(config);
                }
            }

            return uniqueConfigs;
        }

        static string FormatTrackerNames(IReadOnlyList<string> names)
        {
            if (names.Count == 0) return string.Empty;
            if (names.Count <= 3) return string.Join(", ", names);
            return string.Join(", ", names.Take(3)) + $" +{names.Count - 3}";
        }

        static string GetTriggerCaptureStatus()
        {
            lock (TriggerCaptureLock)
            {
                return triggerCaptureStatus;
            }
        }

        static string SaveCurrentConfig()
        {
            string selectedPath = settings.SelectedConfigPath;
            if (string.IsNullOrWhiteSpace(selectedPath))
            {
                return "Save failed: no config selected.";
            }

            var configs = Configs.ToList();
            var savedPaths = new List<string>();

            foreach (string path in GetConfigSavePaths(selectedPath))
            {
                SaveConfigFile(path, configs);
                savedPaths.Add(path);
            }

            return savedPaths.Count == 1
                ? $"Saved: {savedPaths[0]}"
                : $"Saved: {savedPaths[0]} (+ source)";
        }

        static IEnumerable<string> GetConfigSavePaths(string selectedPath)
        {
            string fullPath = Path.GetFullPath(selectedPath);
            yield return fullPath;

            string? sourcePath = ResolveSourceConfigPathFromOutput(fullPath);
            if (!string.IsNullOrWhiteSpace(sourcePath)
                && !string.Equals(Path.GetFullPath(sourcePath), fullPath, StringComparison.OrdinalIgnoreCase))
            {
                yield return sourcePath;
            }
        }

        static string? ResolveSourceConfigPathFromOutput(string configPath)
        {
            string fullPath = Path.GetFullPath(configPath);
            string marker = Path.DirectorySeparatorChar + "bin" + Path.DirectorySeparatorChar;
            int binIndex = fullPath.IndexOf(marker, StringComparison.OrdinalIgnoreCase);
            if (binIndex < 0) return null;

            string projectRoot = fullPath[..binIndex];
            string afterBin = fullPath[(binIndex + marker.Length)..];
            var segments = afterBin.Split(Path.DirectorySeparatorChar, StringSplitOptions.RemoveEmptyEntries);
            if (segments.Length < 3) return null;

            string relativePath = Path.Combine(segments.Skip(2).ToArray());
            return Path.Combine(projectRoot, relativePath);
        }

        static void SaveConfigFile(string configPath, List<BodyPartConfig> configs)
        {
            var lines = File.Exists(configPath)
                ? File.ReadAllLines(configPath).ToList()
                : new List<string>();

            int configIndex = 0;
            for (int i = 0; i < lines.Count; i++)
            {
                string line = StripComment(lines[i]).Trim();
                if (line.Length == 0 || line.Contains('=', StringComparison.Ordinal))
                {
                    continue;
                }

                if (configIndex < configs.Count)
                {
                    lines[i] = SerializeBodyPartConfig(configs[configIndex]);
                    configIndex++;
                }
            }

            while (configIndex < configs.Count)
            {
                lines.Add(SerializeBodyPartConfig(configs[configIndex]));
                configIndex++;
            }

            Directory.CreateDirectory(Path.GetDirectoryName(configPath) ?? AppContext.BaseDirectory);
            File.WriteAllLines(configPath, lines);
        }

        static IEnumerable<string> GetPresetRoots()
        {
            yield return Path.Combine(AppContext.BaseDirectory, "presets");

            string workingPresetRoot = Path.Combine(Directory.GetCurrentDirectory(), "presets");
            if (!string.Equals(workingPresetRoot, Path.Combine(AppContext.BaseDirectory, "presets"), StringComparison.OrdinalIgnoreCase))
            {
                yield return workingPresetRoot;
            }
        }

        static string ResolveDefaultLauncherConfigPath()
        {
            string outputConfigPath = Path.Combine(AppContext.BaseDirectory, "config.txt");
            if (File.Exists(outputConfigPath)) return outputConfigPath;

            string workingConfigPath = Path.Combine(Directory.GetCurrentDirectory(), "config.txt");
            if (File.Exists(workingConfigPath)) return workingConfigPath;

            return outputConfigPath;
        }

        static string ResolvePresetPath(string presetName)
        {
            string safePresetName = presetName.Trim().Trim('\\', '/');
            string outputPresetPath = Path.Combine(AppContext.BaseDirectory, "presets", safePresetName, "config.txt");
            if (File.Exists(outputPresetPath)) return outputPresetPath;

            string workingPresetPath = Path.Combine(Directory.GetCurrentDirectory(), "presets", safePresetName, "config.txt");
            if (File.Exists(workingPresetPath)) return workingPresetPath;

            return outputPresetPath;
        }

        static string ResolvePath(string basePath, string path)
        {
            string trimmedPath = path.Trim().Trim('"');
            return Path.GetFullPath(Path.IsPathRooted(trimmedPath)
                ? trimmedPath
                : Path.Combine(basePath, trimmedPath));
        }

        static string? TryGetArgumentValue(string[] args, string name)
        {
            for (int i = 0; i < args.Length; i++)
            {
                if (args[i].Equals(name, StringComparison.OrdinalIgnoreCase))
                {
                    return i + 1 < args.Length ? args[i + 1] : null;
                }

                string prefix = name + "=";
                if (args[i].StartsWith(prefix, StringComparison.OrdinalIgnoreCase))
                {
                    return args[i][prefix.Length..];
                }
            }

            return null;
        }

        static string? TryReadSelectorValue(string configPath, string key)
        {
            if (!File.Exists(configPath)) return null;

            foreach (var rawLine in File.ReadAllLines(configPath))
            {
                string line = StripComment(rawLine).Trim();
                int settingSeparator = line.IndexOf('=');
                if (settingSeparator <= 0) continue;

                string settingKey = line[..settingSeparator].Trim();
                if (settingKey.Equals(key, StringComparison.OrdinalIgnoreCase))
                {
                    return line[(settingSeparator + 1)..].Trim();
                }
            }

            return null;
        }

        static string StripComment(string line)
        {
            int commentIndex = line.IndexOf('#');
            return commentIndex >= 0 ? line[..commentIndex] : line;
        }

        static void ParseSetting(string key, string value)
        {
            switch (key.Trim().ToLowerInvariant())
            {
                case "fps":
                    settings.Fps = Math.Clamp(int.Parse(value, CultureInfo.InvariantCulture), 0, 2000);
                    break;

                case "space":
                case "trackingspace":
                    settings.TrackingSpace = value.Trim().Equals("room", StringComparison.OrdinalIgnoreCase)
                        ? TrackingSpaceMode.Room
                        : TrackingSpaceMode.Raw;
                    break;

                case "yawonlypositionoffset":
                case "yawonlyoffset":
                    settings.YawOnlyPositionOffset = ParseBool(value);
                    break;

                case "hmdviewpointoffset":
                case "viewpointoffset":
                case "hmdoffset":
                    settings.HmdViewpointOffset = ParseVector(value);
                    break;

                case "host":
                    settings.TargetHost = value.Trim();
                    break;

                case "port":
                    settings.TargetPort = int.Parse(value, CultureInfo.InvariantCulture);
                    break;

                case "timeoffset":
                    settings.TimeOffset = ParseFloat(value);
                    break;

                case "createtrackerswithoutsource":
                case "createwithoutsource":
                case "alwayscreatetrackers":
                    settings.CreateTrackersWithoutSource = ParseBool(value);
                    break;

                case "disabletrackerwhensourcelost":
                case "disablewhensourcelost":
                case "disablewhenlost":
                    settings.DisableTrackerWhenSourceLost = ParseBool(value);
                    break;

                case "predictionms":
                case "posepredictionms":
                    settings.PosePredictionSeconds = Math.Clamp(ParseFloat(value) / 1000f, 0f, 0.05f);
                    break;

                case "predictionseconds":
                case "posepredictionseconds":
                    settings.PosePredictionSeconds = Math.Clamp(ParseFloat(value), 0f, 0.05f);
                    break;

                case "positionpredictionms":
                case "pospredictionms":
                    settings.PositionPredictionSeconds = Math.Clamp(ParseFloat(value) / 1000f, 0f, 0.1f);
                    break;

                case "positionpredictionseconds":
                case "pospredictionseconds":
                    settings.PositionPredictionSeconds = Math.Clamp(ParseFloat(value), 0f, 0.1f);
                    break;

                case "positionpredictionstrength":
                case "positionprediction":
                case "pospredictionstrength":
                    settings.PositionPredictionStrength = ParsePredictionStrength(value);
                    break;

                case "positionpredictionmaxspeed":
                case "pospredictionmaxspeed":
                    settings.PositionPredictionMaxSpeed = Math.Clamp(ParseFloat(value), 0f, 30f);
                    break;

                case "positionvelocitysamplems":
                case "posvelocitysamplems":
                    settings.PositionVelocitySampleSeconds = Math.Clamp(ParseFloat(value) / 1000f, 0.001f, 0.05f);
                    break;

                case "positionvelocityblend":
                case "posvelocityblend":
                    settings.PositionVelocityBlend = Math.Clamp(ParseFloat(value), 0.01f, 1f);
                    break;

                case "positionvelocitydeadzone":
                case "posvelocitydeadzone":
                    settings.PositionVelocityDeadzone = Math.Clamp(ParseFloat(value), 0f, 1f);
                    break;

                case "positiononeeuro":
                case "positiononeeuroenabled":
                case "oneeuro":
                    settings.PositionOneEuroEnabled = ParseBool(value);
                    break;

                case "positiononeeuromincutoff":
                case "oneeuromincutoff":
                    settings.PositionOneEuroMinCutoff = Math.Clamp(ParseFloat(value), 0.01f, 100f);
                    break;

                case "positiononeeurobeta":
                case "oneeurobeta":
                    settings.PositionOneEuroBeta = Math.Clamp(ParseFloat(value), 0f, 100f);
                    break;

                case "positiononeeuroderivativecutoff":
                case "positiononeeurodcutoff":
                case "oneeuroderivativecutoff":
                case "oneeurodcutoff":
                    settings.PositionOneEuroDerivativeCutoff = Math.Clamp(ParseFloat(value), 0.01f, 100f);
                    break;

                case "busywait":
                    settings.BusyWait = ParseBool(value);
                    break;

                case "calibration":
                case "calibrationcontrol":
                    settings.CalibrationControl = ParseCalibrationControlMode(value);
                    break;

                case "calibrationtogglekey":
                case "togglekey":
                    settings.CalibrationToggleKey = ParseVirtualKey(value);
                    break;

                case "calibrationholdkey":
                case "holdkey":
                    settings.CalibrationHoldKey = ParseVirtualKey(value);
                    break;

                case "calibrationyawrotation":
                case "calibrationyaw":
                case "yawrotation":
                    settings.CalibrationYawRotation = ParseBool(value);
                    break;

                case "calibrationlockanchor":
                case "calibrationlockanchorposition":
                case "lockanchor":
                    settings.CalibrationLockAnchorPosition = ParseBool(value);
                    break;

                case "controllerunlockcalibration":
                case "controllerunlock":
                    settings.ControllerUnlockCalibration = ParseBool(value);
                    break;

                case "controllerunlockwithtrigger":
                case "unlockwithtrigger":
                    settings.ControllerUnlockWithTrigger = ParseBool(value);
                    break;

                case "controllerunlockwithgrip":
                case "unlockwithgrip":
                    settings.ControllerUnlockWithGrip = ParseBool(value);
                    break;

                default:
                    Console.WriteLine($"Unknown setting: {key}");
                    break;
            }
        }

        static void ParseBodyPartConfig(string line)
        {
            bool debugOutput = StripDebugSuffix(ref line);
            var parts = line.Split('_');
            if (parts.Length < 4) return;

            try
            {
                if (TryParseBodyPartConfig(line, parts, out var config))
                {
                    config.DebugOutput = debugOutput;
                    Configs.Add(config);
                }
            }
            catch
            {
                Console.WriteLine($"Failed to parse config line: {line}");
            }
        }

        static bool StripDebugSuffix(ref string line)
        {
            string trimmed = line.Trim();
            string[] suffixes = { "＿test", "_test", "＿debug", "_debug" };

            foreach (string suffix in suffixes)
            {
                if (trimmed.EndsWith(suffix, StringComparison.OrdinalIgnoreCase))
                {
                    line = trimmed[..^suffix.Length].TrimEnd();
                    return true;
                }
            }

            line = trimmed;
            return false;
        }

        static bool TryParseBodyPartConfig(string line, string[] parts, out BodyPartConfig config)
        {
            config = new BodyPartConfig();

            if (parts.Length >= 7)
            {
                config = new BodyPartConfig
                {
                    Name = parts[0].Trim(),
                    PositionOffset = ParseVector(parts[1]),
                    RotationOffset = ParseVector(parts[2]),
                    RollOffset = ParseScalar(parts[4]),
                    SerialNumber = parts[5].Trim(),
                    VmtId = int.Parse(parts[6], CultureInfo.InvariantCulture)
                };
                return true;
            }

            if (parts.Length >= 6)
            {
                config = new BodyPartConfig
                {
                    Name = parts[0].Trim(),
                    PositionOffset = ParseVector(parts[1]),
                    RotationOffset = ParseVector(parts[2]),
                    RollOffset = ParseScalar(parts[3]),
                    SerialNumber = parts[4].Trim(),
                    VmtId = int.Parse(parts[5], CultureInfo.InvariantCulture)
                };
                return true;
            }

            if (parts.Length >= 5)
            {
                config = new BodyPartConfig
                {
                    Name = parts[0].Trim(),
                    PositionOffset = ParseVector(parts[1]),
                    RotationOffset = ParseVector(parts[2]),
                    SerialNumber = parts[3].Trim(),
                    VmtId = int.Parse(parts[4], CultureInfo.InvariantCulture)
                };
                return true;
            }

            int separatorIndex = FindVectorSeparator(parts[1]);
            if (separatorIndex <= 0 || separatorIndex >= parts[1].Length - 1)
            {
                Console.WriteLine($"Failed to parse config line: {line}");
                return false;
            }

            config = new BodyPartConfig
            {
                Name = parts[0].Trim(),
                PositionOffset = ParseSlashVector(parts[1][..separatorIndex]),
                RotationOffset = ParseSlashVector(parts[1][(separatorIndex + 1)..]),
                SerialNumber = parts[2].Trim(),
                VmtId = int.Parse(parts[3], CultureInfo.InvariantCulture)
            };
            return true;
        }

        static int FindVectorSeparator(string value)
        {
            for (int i = 1; i < value.Length - 1; i++)
            {
                if (value[i] == '-' && CountChar(value.AsSpan(0, i), '/') == 5 && CountChar(value.AsSpan(i + 1), '/') == 5)
                {
                    return i;
                }
            }

            return -1;
        }

        static int CountChar(ReadOnlySpan<char> value, char target)
        {
            int count = 0;
            foreach (char c in value)
            {
                if (c == target) count++;
            }

            return count;
        }

        static Vector3 ParseVector(string value)
        {
            return value.Contains('/', StringComparison.Ordinal)
                ? ParseSlashVector(value)
                : ParseCommaVector(value);
        }

        static Vector3 ParseCommaVector(string value)
        {
            var values = value.Split(',');
            if (values.Length != 3)
            {
                throw new FormatException("Comma vector must have 3 values.");
            }

            return new Vector3(ParseFloat(values[0]), ParseFloat(values[1]), ParseFloat(values[2]));
        }

        static Vector3 ParseSlashVector(string value)
        {
            var values = value.Split('/');
            if (values.Length != 6)
            {
                throw new FormatException("Slash vector must have 6 values.");
            }

            return new Vector3(
                ParseSlashDecimal(values[0], values[1]),
                ParseSlashDecimal(values[2], values[3]),
                ParseSlashDecimal(values[4], values[5]));
        }

        static float ParseSlashDecimal(string wholePart, string fractionalPart)
        {
            string whole = wholePart.Trim();
            string fraction = fractionalPart.Trim();
            string sign = whole.StartsWith("-", StringComparison.Ordinal) ? "-" : string.Empty;
            string unsignedWhole = sign.Length == 0 ? whole : whole[1..];

            return ParseFloat(sign + unsignedWhole + "." + fraction);
        }

        static float ParseScalar(string value)
        {
            var parts = value.Split('/');
            if (parts.Length == 2)
            {
                return ParseSlashDecimal(parts[0], parts[1]);
            }

            return ParseFloat(value);
        }

        static string SerializeBodyPartConfig(BodyPartConfig config)
        {
            string line;
            if (MathF.Abs(config.RollOffset) > 0.00001f)
            {
                line = string.Join("_",
                    config.Name,
                    FormatSlashVector(config.PositionOffset),
                    FormatSlashVector(config.RotationOffset),
                    FormatSlashDecimal(config.RollOffset),
                    config.SerialNumber,
                    config.VmtId.ToString(CultureInfo.InvariantCulture));

                return config.DebugOutput ? line + "＿test" : line;
            }

            line = string.Join("_",
                config.Name,
                FormatSlashVector(config.PositionOffset),
                FormatSlashVector(config.RotationOffset),
                config.SerialNumber,
                config.VmtId.ToString(CultureInfo.InvariantCulture));

            return config.DebugOutput ? line + "＿test" : line;
        }

        static string FormatSlashVector(Vector3 value)
        {
            return string.Join("/",
                FormatSlashDecimal(value.X),
                FormatSlashDecimal(value.Y),
                FormatSlashDecimal(value.Z));
        }

        static string FormatSlashDecimal(float value)
        {
            string formatted = value.ToString("0.######", CultureInfo.InvariantCulture);
            int decimalIndex = formatted.IndexOf('.');
            if (decimalIndex < 0)
            {
                return formatted + "/0";
            }

            return formatted[..decimalIndex] + "/" + formatted[(decimalIndex + 1)..];
        }

        static void ApplySettings()
        {
            if (settings.TrackingSpace == TrackingSpaceMode.Room)
            {
                trackingOrigin = ETrackingUniverseOrigin.TrackingUniverseStanding;
                oscAddress = "/VMT/Room/Unity";
            }
            else
            {
                trackingOrigin = ETrackingUniverseOrigin.TrackingUniverseRawAndUncalibrated;
                oscAddress = "/VMT/Raw/Unity";
            }
        }

        static void StartControlUi()
        {
            controlUiThread = new Thread(() =>
            {
                var form = new CalibrationForm(
                    () => calibrationActive,
                    SetCalibrationActive,
                    FindPresetChoices,
                    () => settings.SelectedConfigPath,
                    GetBodyConfigs,
                    CaptureCurrentPoses,
                    ArmTriggerCapture,
                    GetTriggerCaptureStatus,
                    SaveCurrentConfig,
                    () => settings.PositionPredictionStrength,
                    () => EffectivePositionPredictionSeconds() * 1000f,
                    strength => settings.PositionPredictionStrength = Math.Clamp(strength, 0f, 1f),
                    RequestConfigReload,
                    () => exitRequested = true);
                calibrationForm = form;
                Application.Run(form);
            });

            controlUiThread.IsBackground = true;
            controlUiThread.TrySetApartmentState(ApartmentState.STA);
            controlUiThread.Start();
        }

        static void ShutdownControlUi()
        {
            var form = calibrationForm;
            if (form != null && !form.IsDisposed)
            {
                try
                {
                    form.BeginInvoke((Action)(() => form.Close()));
                }
                catch
                {
                    // The control window may already be closing.
                }
            }

            controlUiThread?.Join(1000);
        }

        static void SetCalibrationActive(bool active)
        {
            calibrationActive = active;
        }

        static void RequestConfigReload(string configPath)
        {
            Interlocked.Exchange(ref pendingConfigReloadPath, configPath);
        }

        static bool TryConsumePendingConfigReload(out string configPath)
        {
            configPath = Interlocked.Exchange(ref pendingConfigReloadPath, null) ?? string.Empty;
            return configPath.Length > 0;
        }

        static bool ApplyConfigReload(string configPath)
        {
            if (!File.Exists(configPath))
            {
                Console.WriteLine($"Preset switch failed: config.txt not found. ({configPath})");
                return false;
            }

            SendDisableAll(force: true);
            calibrationActive = false;
            toggleKeyWasDown = false;
            ResetCalibrationAnchor();

            if (!LoadConfig(configPath))
            {
                return false;
            }

            Sender.Connect(settings.TargetHost, settings.TargetPort);
            SendCreateAllAtOrigin(force: true);
            Console.WriteLine($"Switched preset: {settings.SelectedConfigPath}");
            return true;
        }

        static void UpdateCalibrationInputState()
        {
            if (settings.CalibrationControl == CalibrationControlMode.Off)
            {
                calibrationActive = false;
                return;
            }

            if (settings.CalibrationControl == CalibrationControlMode.HoldKey)
            {
                calibrationActive = NativeMethods.IsKeyDown(settings.CalibrationHoldKey);
                return;
            }

            bool toggleKeyDown = NativeMethods.IsKeyDown(settings.CalibrationToggleKey);
            if (toggleKeyDown && !toggleKeyWasDown)
            {
                calibrationActive = !calibrationActive;
            }

            toggleKeyWasDown = toggleKeyDown;
        }

        static Vector3 UpdateCalibrationReferencePosition(Vector3 hmdPosition, Quaternion hmdRotation, Quaternion baseRotation, bool isCalibrating)
        {
            Vector3 correctedHmdReferencePosition = GetHmdReferencePosition(hmdPosition, hmdRotation);
            Vector3 referencePosition = correctedHmdReferencePosition;

            lock (PoseStateLock)
            {
                latestHmdPosition = correctedHmdReferencePosition;
                latestHmdRotation = hmdRotation;
                latestCalibrationBaseRotation = baseRotation;
                hasLatestHmdPose = true;

                if (!isCalibrating)
                {
                    calibrationAnchorPositionValid = false;
                    lastCalibrationActiveState = false;
                }
                else if (settings.CalibrationLockAnchorPosition)
                {
                    if (!lastCalibrationActiveState || !calibrationAnchorPositionValid)
                    {
                        calibrationAnchorPosition = correctedHmdReferencePosition;
                        calibrationAnchorPositionValid = true;
                    }

                    referencePosition = calibrationAnchorPosition;
                }

                lastCalibrationActiveState = isCalibrating;
                latestCalibrationReferencePosition = referencePosition;
                hasLatestCalibrationReferencePosition = true;
            }

            return referencePosition;
        }

        static void ResetCalibrationAnchor()
        {
            lock (PoseStateLock)
            {
                hasLatestCalibrationReferencePosition = false;
                latestCalibrationBaseRotation = Quaternion.Identity;
                calibrationAnchorPositionValid = false;
                lastCalibrationActiveState = false;
            }
        }

        static void UpdateTriggerCapture(CVRSystem vrSystem)
        {
            bool triggerDown = TryGetAnyControllerTriggerDown(vrSystem, out uint controllerIndex);
            bool gripDown = TryGetAnyControllerGripDown(vrSystem, out uint gripControllerIndex);
            List<BodyPartConfig>? targets = null;
            bool unlockCalibration = false;
            uint feedbackControllerIndex = OpenVR.k_unTrackedDeviceIndexInvalid;

            lock (TriggerCaptureLock)
            {
                bool triggerPressed = triggerDown && !controllerTriggerWasDown;
                bool gripPressed = gripDown && !controllerGripWasDown;

                if (triggerCaptureArmed)
                {
                    if (triggerCaptureIgnoreUntilReleased)
                    {
                        if (!triggerDown)
                        {
                            triggerCaptureIgnoreUntilReleased = false;
                        }
                    }
                    else if (triggerPressed)
                    {
                        targets = TriggerCaptureTargets.ToList();
                        triggerCaptureArmed = false;
                        TriggerCaptureTargets.Clear();
                        feedbackControllerIndex = controllerIndex;
                    }
                }
                else if (settings.ControllerUnlockCalibration && calibrationActive)
                {
                    bool shouldUnlock =
                        (settings.ControllerUnlockWithTrigger && triggerPressed)
                        || (settings.ControllerUnlockWithGrip && gripPressed);

                    if (shouldUnlock)
                    {
                        calibrationActive = false;
                        unlockCalibration = true;
                        triggerCaptureStatus = "Calibration unlocked";
                        feedbackControllerIndex = triggerPressed ? controllerIndex : gripControllerIndex;
                    }
                }

                controllerTriggerWasDown = triggerDown;
                controllerGripWasDown = gripDown;
            }

            if (targets != null)
            {
                string result = CaptureCurrentPoses(targets);
                lock (TriggerCaptureLock)
                {
                    triggerCaptureStatus = result;
                }
            }

            if (targets == null && !unlockCalibration)
            {
                return;
            }

            if (feedbackControllerIndex != OpenVR.k_unTrackedDeviceIndexInvalid)
            {
                try
                {
                    vrSystem.TriggerHapticPulse(feedbackControllerIndex, 0, 800);
                }
                catch
                {
                    // Capture feedback is optional; the pose data has already been recorded.
                }
            }
        }

        static bool TryGetAnyControllerTriggerDown(CVRSystem vrSystem, out uint controllerIndex)
        {
            controllerIndex = OpenVR.k_unTrackedDeviceIndexInvalid;

            for (uint i = 0; i < OpenVR.k_unMaxTrackedDeviceCount; i++)
            {
                if (!vrSystem.IsTrackedDeviceConnected(i)) continue;
                if (vrSystem.GetTrackedDeviceClass(i) != ETrackedDeviceClass.Controller) continue;

                var state = new VRControllerState_t();
                if (!vrSystem.GetControllerState(i, ref state, (uint)System.Runtime.InteropServices.Marshal.SizeOf<VRControllerState_t>()))
                {
                    continue;
                }

                if (IsTriggerPressed(state))
                {
                    controllerIndex = i;
                    return true;
                }
            }

            return false;
        }

        static bool TryGetAnyControllerGripDown(CVRSystem vrSystem, out uint controllerIndex)
        {
            controllerIndex = OpenVR.k_unTrackedDeviceIndexInvalid;

            for (uint i = 0; i < OpenVR.k_unMaxTrackedDeviceCount; i++)
            {
                if (!vrSystem.IsTrackedDeviceConnected(i)) continue;
                if (vrSystem.GetTrackedDeviceClass(i) != ETrackedDeviceClass.Controller) continue;

                var state = new VRControllerState_t();
                if (!vrSystem.GetControllerState(i, ref state, (uint)System.Runtime.InteropServices.Marshal.SizeOf<VRControllerState_t>()))
                {
                    continue;
                }

                if (IsGripPressed(state))
                {
                    controllerIndex = i;
                    return true;
                }
            }

            return false;
        }

        static bool IsTriggerPressed(VRControllerState_t state)
        {
            const ulong triggerButtonMask = 1UL << (int)EVRButtonId.k_EButton_SteamVR_Trigger;
            return (state.ulButtonPressed & triggerButtonMask) != 0
                || state.rAxis1.x > 0.75f
                || state.rAxis1.y > 0.75f;
        }

        static bool IsGripPressed(VRControllerState_t state)
        {
            const ulong gripButtonMask = 1UL << (int)EVRButtonId.k_EButton_Grip;
            return (state.ulButtonPressed & gripButtonMask) != 0;
        }

        static void ApplyRuntimeHints()
        {
            try
            {
                using var process = Process.GetCurrentProcess();
                process.PriorityClass = ProcessPriorityClass.High;
                Thread.CurrentThread.Priority = ThreadPriority.Highest;
            }
            catch
            {
                // Pose sending can continue even when priority changes are unavailable.
            }
        }

        static CalibrationControlMode ParseCalibrationControlMode(string value)
        {
            string normalized = value.Trim().ToLowerInvariant();
            return normalized switch
            {
                "gui" => CalibrationControlMode.Gui,
                "toggle" or "togglekey" or "key" => CalibrationControlMode.ToggleKey,
                "hold" or "holdkey" => CalibrationControlMode.HoldKey,
                "off" or "none" or "false" or "0" => CalibrationControlMode.Off,
                _ => CalibrationControlMode.Gui
            };
        }

        static int ParseVirtualKey(string value)
        {
            string normalized = value.Trim().ToUpperInvariant();
            return normalized switch
            {
                "SPACE" => 0x20,
                "ESC" or "ESCAPE" => 0x1B,
                "F1" => 0x70,
                "F2" => 0x71,
                "F3" => 0x72,
                "F4" => 0x73,
                "F5" => 0x74,
                "F6" => 0x75,
                "F7" => 0x76,
                "F8" => 0x77,
                "F9" => 0x78,
                "F10" => 0x79,
                "F11" => 0x7A,
                "F12" => 0x7B,
                _ when normalized.Length == 1 => normalized[0],
                _ when normalized.StartsWith("0X", StringComparison.Ordinal) => Convert.ToInt32(normalized, 16),
                _ => int.Parse(normalized, CultureInfo.InvariantCulture)
            };
        }

        static string KeyName(int virtualKey)
        {
            return virtualKey switch
            {
                0x20 => "Space",
                0x1B => "Esc",
                >= 0x70 and <= 0x7B => "F" + (virtualKey - 0x6F).ToString(CultureInfo.InvariantCulture),
                _ when virtualKey >= 'A' && virtualKey <= 'Z' => ((char)virtualKey).ToString(),
                _ => "0x" + virtualKey.ToString("X2", CultureInfo.InvariantCulture)
            };
        }

        static bool ParseBool(string value)
        {
            string normalized = value.Trim().ToLowerInvariant();
            return normalized is "1" or "true" or "yes" or "on";
        }

        static float ParsePredictionStrength(string value)
        {
            string trimmed = value.Trim();
            bool percentStyle = trimmed.EndsWith("%", StringComparison.Ordinal);
            if (percentStyle)
            {
                trimmed = trimmed[..^1];
            }

            float parsed = ParseFloat(trimmed);
            if (percentStyle || parsed > 1f)
            {
                parsed /= 100f;
            }

            return Math.Clamp(parsed, 0f, 1f);
        }

        static float ParseFloat(string value)
        {
            return float.Parse(value.Trim(), CultureInfo.InvariantCulture);
        }

        static float EffectivePositionPredictionSeconds()
        {
            return Math.Clamp(settings.PositionPredictionSeconds * settings.PositionPredictionStrength, 0f, 0.1f);
        }

        static void EnsureDeviceIndex(CVRSystem system, TrackedDevicePose_t[] poses, BodyPartConfig conf, long now)
        {
            if (conf.DeviceIndex != OpenVR.k_unTrackedDeviceIndexInvalid)
            {
                bool stillConnected = conf.DeviceIndex < poses.Length
                    && system.IsTrackedDeviceConnected(conf.DeviceIndex);

                if (stillConnected) return;
                conf.DeviceIndex = OpenVR.k_unTrackedDeviceIndexInvalid;
            }

            if (now < conf.NextDeviceSearchTick) return;

            conf.DeviceIndex = FindDeviceBySerial(system, conf.SerialNumber);
            conf.NextDeviceSearchTick = now + Stopwatch.Frequency;
        }

        static Vector3 GetUnityPosition(HmdMatrix34_t pose)
        {
            return new Vector3(pose.m3, pose.m7, -pose.m11);
        }

        static Quaternion CreateUnityQuaternionFromEuler(Vector3 euler)
        {
            float rad = (float)(Math.PI / 180.0);
            Quaternion qx = Quaternion.CreateFromAxisAngle(Vector3.UnitX, euler.X * rad);
            Quaternion qy = Quaternion.CreateFromAxisAngle(Vector3.UnitY, euler.Y * rad);
            Quaternion qz = Quaternion.CreateFromAxisAngle(Vector3.UnitZ, euler.Z * rad);

            return Quaternion.Normalize(qy * qx * qz);
        }

        static Quaternion CreateCalibrationOffsetRotation(BodyPartConfig config)
        {
            Quaternion rotation = CreateUnityQuaternionFromEuler(config.RotationOffset);
            if (MathF.Abs(config.RollOffset) <= 0.0001f)
            {
                return rotation;
            }

            float rad = (float)(Math.PI / 180.0);
            Quaternion rollOffset = Quaternion.CreateFromAxisAngle(Vector3.UnitZ, config.RollOffset * rad);
            return Quaternion.Normalize(rotation * rollOffset);
        }

        static Quaternion GetCalibrationBaseRotation(HmdMatrix34_t pose, Quaternion hmdRotation)
        {
            return settings.YawOnlyPositionOffset ? GetYawOnlyRotation(pose) : hmdRotation;
        }

        static Vector3 GetHmdReferencePosition(Vector3 hmdPosition, Quaternion hmdRotation)
        {
            if (settings.HmdViewpointOffset.LengthSquared() <= 0.0000001f)
            {
                return hmdPosition;
            }

            return hmdPosition + Vector3.Transform(settings.HmdViewpointOffset, hmdRotation);
        }

        static Vector3 GetUnityEulerFromQuaternion(Quaternion rotation)
        {
            if (rotation.LengthSquared() < 0.000001f)
            {
                return Vector3.Zero;
            }

            rotation = Quaternion.Normalize(rotation);
            Vector3 forward = Vector3.Transform(Vector3.UnitZ, rotation);
            Vector3 right = Vector3.Transform(Vector3.UnitX, rotation);
            Vector3 up = Vector3.Transform(Vector3.UnitY, rotation);

            float pitch = MathF.Asin(Math.Clamp(-forward.Y, -1f, 1f));
            float yaw;
            float roll;

            if (MathF.Abs(MathF.Cos(pitch)) > 0.0001f)
            {
                yaw = MathF.Atan2(forward.X, forward.Z);
                roll = MathF.Atan2(right.Y, up.Y);
            }
            else
            {
                yaw = MathF.Atan2(-right.Z, right.X);
                roll = 0f;
            }

            const float degrees = 180f / MathF.PI;
            return new Vector3(
                NormalizeDegrees(pitch * degrees),
                NormalizeDegrees(yaw * degrees),
                NormalizeDegrees(roll * degrees));
        }

        static float NormalizeDegrees(float value)
        {
            while (value > 180f) value -= 360f;
            while (value < -180f) value += 360f;
            return value;
        }

        static Quaternion GetUnityRotation(HmdMatrix34_t pose)
        {
            if (!IsRotationValid(pose)) return Quaternion.Identity;

            float w = MathF.Sqrt(MathF.Max(0f, 1f + pose.m0 + pose.m5 + pose.m10)) / 2f;
            float x = MathF.Sqrt(MathF.Max(0f, 1f + pose.m0 - pose.m5 - pose.m10)) / 2f;
            float y = MathF.Sqrt(MathF.Max(0f, 1f - pose.m0 + pose.m5 - pose.m10)) / 2f;
            float z = MathF.Sqrt(MathF.Max(0f, 1f - pose.m0 - pose.m5 + pose.m10)) / 2f;

            CopySign(ref x, pose.m6 - pose.m9);
            CopySign(ref y, pose.m8 - pose.m2);
            CopySign(ref z, pose.m4 - pose.m1);

            return Quaternion.Normalize(new Quaternion(x, y, z, w));
        }

        static Quaternion GetYawOnlyRotation(Quaternion rotation)
        {
            Vector3 forward = Vector3.Transform(Vector3.UnitZ, rotation);
            forward.Y = 0f;

            if (forward.LengthSquared() < 0.000001f)
            {
                return Quaternion.Identity;
            }

            forward = Vector3.Normalize(forward);
            float yaw = MathF.Atan2(forward.X, forward.Z);
            return Quaternion.CreateFromAxisAngle(Vector3.UnitY, yaw);
        }

        static Quaternion GetYawOnlyRotation(HmdMatrix34_t pose)
        {
            Vector3 forward = new(-pose.m2, 0f, pose.m10);

            if (forward.LengthSquared() < 0.000001f)
            {
                return Quaternion.Identity;
            }

            forward = Vector3.Normalize(forward);
            float yaw = MathF.Atan2(forward.X, forward.Z);
            return Quaternion.CreateFromAxisAngle(Vector3.UnitY, yaw);
        }

        static bool IsRotationValid(HmdMatrix34_t pose)
        {
            return (pose.m2 != 0f || pose.m6 != 0f || pose.m10 != 0f)
                && (pose.m1 != 0f || pose.m5 != 0f || pose.m9 != 0f);
        }

        static Vector3 ApplyPositionPrediction(BodyPartConfig conf, Vector3 position, long tick)
        {
            Vector3 velocity = UpdatePositionVelocity(conf, position, tick);
            float predictionSeconds = EffectivePositionPredictionSeconds();
            if (predictionSeconds <= 0f || velocity.LengthSquared() <= 0.000001f)
            {
                conf.PredictionOffsetFilter.Reset();
                return position;
            }

            Vector3 predictionOffset = velocity * predictionSeconds;
            if (settings.PositionOneEuroEnabled)
            {
                predictionOffset = conf.PredictionOffsetFilter.Filter(
                    predictionOffset,
                    tick / (double)Stopwatch.Frequency,
                    settings.PositionOneEuroMinCutoff,
                    settings.PositionOneEuroBeta,
                    settings.PositionOneEuroDerivativeCutoff);
            }

            return position + predictionOffset;
        }

        static Vector3 UpdatePositionVelocity(BodyPartConfig conf, Vector3 position, long tick)
        {
            if (!conf.HasLastPosition || tick <= conf.LastPositionTick)
            {
                conf.LastPosition = position;
                conf.LastPositionTick = tick;
                conf.Velocity = Vector3.Zero;
                conf.HasLastPosition = true;
                return Vector3.Zero;
            }

            float deltaSeconds = (float)((tick - conf.LastPositionTick) / (double)Stopwatch.Frequency);
            if (deltaSeconds <= 0f || deltaSeconds > 0.25f)
            {
                conf.LastPosition = position;
                conf.LastPositionTick = tick;
                conf.Velocity = Vector3.Zero;
                return Vector3.Zero;
            }

            if (deltaSeconds < settings.PositionVelocitySampleSeconds)
            {
                return ApplyVelocityDeadzone(conf.Velocity);
            }

            Vector3 rawVelocity = (position - conf.LastPosition) / deltaSeconds;
            if (!IsFinite(rawVelocity))
            {
                rawVelocity = Vector3.Zero;
            }

            rawVelocity = ClampVelocity(rawVelocity, settings.PositionPredictionMaxSpeed);
            conf.Velocity = Vector3.Lerp(conf.Velocity, rawVelocity, settings.PositionVelocityBlend);
            conf.Velocity = ClampVelocity(conf.Velocity, settings.PositionPredictionMaxSpeed);
            conf.LastPosition = position;
            conf.LastPositionTick = tick;
            return ApplyVelocityDeadzone(conf.Velocity);
        }

        static Vector3 ClampVelocity(Vector3 velocity, float maxSpeed)
        {
            if (maxSpeed <= 0f)
            {
                return velocity;
            }

            float maxSpeedSquared = maxSpeed * maxSpeed;
            float speedSquared = velocity.LengthSquared();
            if (speedSquared <= maxSpeedSquared)
            {
                return velocity;
            }

            return Vector3.Normalize(velocity) * maxSpeed;
        }

        static Vector3 ApplyVelocityDeadzone(Vector3 velocity)
        {
            float deadzone = settings.PositionVelocityDeadzone;
            if (deadzone <= 0f)
            {
                return velocity;
            }

            float speed = velocity.Length();
            if (speed <= deadzone)
            {
                return Vector3.Zero;
            }

            return velocity * ((speed - deadzone) / speed);
        }

        static void ResetPositionPrediction(BodyPartConfig conf)
        {
            conf.HasLastPosition = false;
            conf.LastPosition = Vector3.Zero;
            conf.Velocity = Vector3.Zero;
            conf.PredictionOffsetFilter.Reset();
            conf.LastPositionTick = 0;
        }

        static bool IsFinite(Vector3 value)
        {
            return float.IsFinite(value.X)
                && float.IsFinite(value.Y)
                && float.IsFinite(value.Z);
        }

        static void CopySign(ref float value, float sign)
        {
            if ((sign > 0f) != (value > 0f))
            {
                value = -value;
            }
        }

        static uint FindDeviceBySerial(CVRSystem system, string serial)
        {
            if (string.IsNullOrWhiteSpace(serial))
            {
                return OpenVR.k_unTrackedDeviceIndexInvalid;
            }

            for (uint i = 0; i < OpenVR.k_unMaxTrackedDeviceCount; i++)
            {
                if (!system.IsTrackedDeviceConnected(i)) continue;

                var sb = new StringBuilder(64);
                var error = ETrackedPropertyError.TrackedProp_Success;
                system.GetStringTrackedDeviceProperty(i, ETrackedDeviceProperty.Prop_SerialNumber_String, sb, 64, ref error);
                if (error == ETrackedPropertyError.TrackedProp_Success && sb.ToString().Trim() == serial.Trim())
                {
                    return i;
                }
            }

            return OpenVR.k_unTrackedDeviceIndexInvalid;
        }

        static void SendDisableAll(bool force)
        {
            foreach (var conf in Configs)
            {
                ResetPositionPrediction(conf);
                SendPose(conf, enable: false, Vector3.Zero, Quaternion.Identity, force);
            }
        }

        static void SendCreateAllAtOrigin(bool force)
        {
            if (!settings.CreateTrackersWithoutSource)
            {
                SendDisableAll(force);
                return;
            }

            foreach (var conf in Configs)
            {
                ResetPositionPrediction(conf);
                bool enable = !settings.DisableTrackerWhenSourceLost || !conf.HasEverHadSourcePose;
                SendPose(conf, enable, Vector3.Zero, Quaternion.Identity, force);
            }
        }

        static void SendPose(BodyPartConfig conf, bool enable, Vector3 position, Quaternion rotation, bool force = false)
        {
            if (!force && !enable && !conf.LastEnableSent) return;

            int length = BuildVmtPoseMessage(
                oscAddress,
                conf.VmtId,
                enable ? 1 : 0,
                settings.TimeOffset,
                position,
                rotation,
                OscBuffer);

            Sender.Send(OscBuffer, length);
            conf.LastEnableSent = enable;
        }

        static int BuildVmtPoseMessage(string address, int index, int enable, float timeOffset, Vector3 position, Quaternion rotation, byte[] buffer)
        {
            int offset = 0;
            WriteOscString(buffer, ref offset, address);
            WriteOscString(buffer, ref offset, TypeTag);
            WriteInt32(buffer, ref offset, index);
            WriteInt32(buffer, ref offset, enable);
            WriteFloat32(buffer, ref offset, timeOffset);
            WriteFloat32(buffer, ref offset, position.X);
            WriteFloat32(buffer, ref offset, position.Y);
            WriteFloat32(buffer, ref offset, position.Z);
            WriteFloat32(buffer, ref offset, rotation.X);
            WriteFloat32(buffer, ref offset, rotation.Y);
            WriteFloat32(buffer, ref offset, rotation.Z);
            WriteFloat32(buffer, ref offset, rotation.W);
            return offset;
        }

        static void WriteOscString(byte[] buffer, ref int offset, string value)
        {
            offset += Encoding.ASCII.GetBytes(value, 0, value.Length, buffer, offset);
            buffer[offset++] = 0;

            while ((offset & 3) != 0)
            {
                buffer[offset++] = 0;
            }
        }

        static void WriteInt32(byte[] buffer, ref int offset, int value)
        {
            unchecked
            {
                buffer[offset++] = (byte)(value >> 24);
                buffer[offset++] = (byte)(value >> 16);
                buffer[offset++] = (byte)(value >> 8);
                buffer[offset++] = (byte)value;
            }
        }

        static void WriteFloat32(byte[] buffer, ref int offset, float value)
        {
            int bits = BitConverter.SingleToInt32Bits(value);
            WriteInt32(buffer, ref offset, bits);
        }

        static void WaitUntil(long targetTick)
        {
            while (true)
            {
                long remainingTicks = targetTick - Stopwatch.GetTimestamp();
                if (remainingTicks <= 0) return;

                double remainingMs = remainingTicks * 1000.0 / Stopwatch.Frequency;
                if (!settings.BusyWait && remainingMs > 4.0)
                {
                    Thread.Sleep(1);
                }
                else if (!settings.BusyWait && remainingMs > 0.25)
                {
                    Thread.Sleep(0);
                }
                else
                {
                    Thread.SpinWait(64);
                }
            }
        }

        static void PrintStats(double actualFps, int sentCount, double avgPoseMs, double avgWaitMs, Vector3 hmdPos, bool hmdValid, bool isCalibrating)
        {
            try
            {
                Console.SetCursorPosition(0, 8);
            }
            catch
            {
                return;
            }

            int lineCount = 0;

            WriteStatsLine($"FPS: {actualFps,7:F1} / Target: {(settings.Fps > 0 ? settings.Fps.ToString(CultureInfo.InvariantCulture) : "Unlimited"),9}  Sent: {sentCount,5}  Pred: {settings.PosePredictionSeconds * 1000f,4:F1}ms  PosPred: {settings.PositionPredictionStrength * 100f,3:F0}%/{EffectivePositionPredictionSeconds() * 1000f,4:F1}ms  Busy: {settings.BusyWait}  Mode: {(isCalibrating ? "CALIB" : "LIVE ")}");
            lineCount++;
            WriteStatsLine($"Avg: Pose {avgPoseMs,6:F3}ms  Wait {avgWaitMs,6:F3}ms");
            lineCount++;
            WriteStatsLine($"HMD: {(hmdValid ? "OK " : "NG ")} Pos: {hmdPos.X,7:F3}, {hmdPos.Y,7:F3}, {hmdPos.Z,7:F3}  Space: {settings.TrackingSpace}");
            lineCount++;

            foreach (var conf in Configs)
            {
                string indexText = conf.DeviceIndex == OpenVR.k_unTrackedDeviceIndexInvalid ? "-" : conf.DeviceIndex.ToString(CultureInfo.InvariantCulture);
                WriteStatsLine($"{conf.Name}: VMT {conf.VmtId} / OpenVR {indexText} / {(conf.LastEnableSent ? "Enabled " : "Disabled")}");
                lineCount++;
            }

            foreach (string line in BuildDebugOutputLines())
            {
                WriteStatsLine(line);
                lineCount++;
            }

            for (int i = lineCount; i < lastStatsLineCount; i++)
            {
                WriteStatsLine(string.Empty);
            }

            lastStatsLineCount = lineCount;
        }

        static List<string> BuildDebugOutputLines()
        {
            var debugConfigs = Configs.Where(config => config.DebugOutput).ToList();
            var lines = new List<string>();
            if (debugConfigs.Count == 0)
            {
                return lines;
            }

            lock (PoseStateLock)
            {
                foreach (var config in debugConfigs)
                {
                    if (!hasLatestHmdPose)
                    {
                        lines.Add($"TEST {config.Name}: HMD pose invalid");
                        continue;
                    }

                    if (!config.HasLatestLivePose)
                    {
                        lines.Add($"TEST {config.Name}: tracker pose invalid");
                        continue;
                    }

                    Quaternion inverseBaseRot = Quaternion.Inverse(latestCalibrationBaseRotation);
                    Vector3 referencePosition = hasLatestCalibrationReferencePosition
                        ? latestCalibrationReferencePosition
                        : latestHmdPosition;
                    Vector3 positionOffset = Vector3.Transform(config.LatestLivePosition - referencePosition, inverseBaseRot);
                    Quaternion rotationOffset = settings.CalibrationYawRotation
                        ? Quaternion.Normalize(inverseBaseRot * config.LatestLiveRotation)
                        : Quaternion.Normalize(config.LatestLiveRotation);
                    Vector3 rotationEuler = GetUnityEulerFromQuaternion(rotationOffset);

                    lines.Add($"TEST {config.Name}: {FormatSlashVector(positionOffset)}_{FormatSlashVector(rotationEuler)}");
                }
            }

            return lines;
        }

        static void WriteStatsLine(string value)
        {
            try
            {
                int width = Math.Max(1, Console.BufferWidth - 1);
                if (value.Length > width)
                {
                    value = value[..width];
                }
                else
                {
                    value = value.PadRight(width);
                }
            }
            catch
            {
                // Some console hosts do not expose BufferWidth; plain output is fine there.
            }

            Console.WriteLine(value);
        }
    }

    class CalibrationForm : Form
    {
        readonly Func<bool> getCalibrationActive;
        readonly Action<bool> setCalibrationActive;
        readonly Func<List<PresetChoice>> getPresetChoices;
        readonly Func<string> getCurrentConfigPath;
        readonly Func<List<BodyPartConfig>> getBodyConfigs;
        readonly Func<IReadOnlyList<BodyPartConfig>, string> captureCurrentPoses;
        readonly Func<IReadOnlyList<BodyPartConfig>, string> armTriggerCapture;
        readonly Func<string> getTriggerCaptureStatus;
        readonly Func<string> saveCurrentConfig;
        readonly Func<float> getPositionPredictionStrength;
        readonly Func<float> getEffectivePositionPredictionMs;
        readonly Action<float> setPositionPredictionStrength;
        readonly Action<string> requestConfigReload;
        readonly Action requestExit;
        readonly ComboBox presetComboBox;
        readonly ComboBox bodyConfigComboBox;
        readonly CheckedListBox batchTrackerListBox;
        readonly TextBox selectedPathTextBox;
        readonly TextBox currentPathTextBox;
        readonly NumericUpDown positionXInput;
        readonly NumericUpDown positionYInput;
        readonly NumericUpDown positionZInput;
        readonly NumericUpDown rotationXInput;
        readonly NumericUpDown rotationYInput;
        readonly NumericUpDown rotationZInput;
        readonly NumericUpDown rollOffsetInput;
        readonly TrackBar positionPredictionTrackBar;
        readonly Label positionPredictionValueLabel;
        readonly Button applyPresetButton;
        readonly Button refreshPresetButton;
        readonly Button capturePoseButton;
        readonly Button armTriggerButton;
        readonly Button applyCheckedButton;
        readonly Button saveConfigButton;
        readonly Button selectAllTrackersButton;
        readonly Button selectNoTrackersButton;
        readonly Button toggleButton;
        readonly Label saveStatusLabel;
        readonly Label statusLabel;
        readonly System.Windows.Forms.Timer refreshTimer;
        string lastBodyConfigPath = string.Empty;
        int lastBodyConfigCount = -1;
        string lastTriggerStatus = string.Empty;
        bool refreshingControls;

        public CalibrationForm(
            Func<bool> getCalibrationActive,
            Action<bool> setCalibrationActive,
            Func<List<PresetChoice>> getPresetChoices,
            Func<string> getCurrentConfigPath,
            Func<List<BodyPartConfig>> getBodyConfigs,
            Func<IReadOnlyList<BodyPartConfig>, string> captureCurrentPoses,
            Func<IReadOnlyList<BodyPartConfig>, string> armTriggerCapture,
            Func<string> getTriggerCaptureStatus,
            Func<string> saveCurrentConfig,
            Func<float> getPositionPredictionStrength,
            Func<float> getEffectivePositionPredictionMs,
            Action<float> setPositionPredictionStrength,
            Action<string> requestConfigReload,
            Action requestExit)
        {
            this.getCalibrationActive = getCalibrationActive;
            this.setCalibrationActive = setCalibrationActive;
            this.getPresetChoices = getPresetChoices;
            this.getCurrentConfigPath = getCurrentConfigPath;
            this.getBodyConfigs = getBodyConfigs;
            this.captureCurrentPoses = captureCurrentPoses;
            this.armTriggerCapture = armTriggerCapture;
            this.getTriggerCaptureStatus = getTriggerCaptureStatus;
            this.saveCurrentConfig = saveCurrentConfig;
            this.getPositionPredictionStrength = getPositionPredictionStrength;
            this.getEffectivePositionPredictionMs = getEffectivePositionPredictionMs;
            this.setPositionPredictionStrength = setPositionPredictionStrength;
            this.requestConfigReload = requestConfigReload;
            this.requestExit = requestExit;

            Text = "VMT SETO Control";
            Width = 620;
            Height = 740;
            FormBorderStyle = FormBorderStyle.FixedSingle;
            MaximizeBox = false;
            TopMost = true;
            StartPosition = FormStartPosition.CenterScreen;

            var presetLabel = new Label
            {
                Left = 16,
                Top = 16,
                Width = 560,
                Height = 20,
                Text = "Preset"
            };

            presetComboBox = new ComboBox
            {
                Left = 16,
                Top = 40,
                Width = 396,
                Height = 24,
                DropDownStyle = ComboBoxStyle.DropDownList
            };
            presetComboBox.SelectedIndexChanged += (_, _) => RefreshSelectedPath();

            refreshPresetButton = new Button
            {
                Left = 420,
                Top = 38,
                Width = 80,
                Height = 28,
                Text = "Refresh"
            };
            refreshPresetButton.Click += (_, _) => LoadPresetChoices();

            applyPresetButton = new Button
            {
                Left = 508,
                Top = 38,
                Width = 80,
                Height = 28,
                Text = "Apply"
            };
            applyPresetButton.Click += (_, _) => ApplySelectedPreset();

            selectedPathTextBox = new TextBox
            {
                Left = 16,
                Top = 72,
                Width = 572,
                Height = 24,
                ReadOnly = true
            };

            var currentLabel = new Label
            {
                Left = 16,
                Top = 108,
                Width = 560,
                Height = 20,
                Text = "Current"
            };

            currentPathTextBox = new TextBox
            {
                Left = 16,
                Top = 132,
                Width = 572,
                Height = 24,
                ReadOnly = true
            };

            var bodyLabel = new Label
            {
                Left = 16,
                Top = 164,
                Width = 560,
                Height = 20,
                Text = "Tracker"
            };

            bodyConfigComboBox = new ComboBox
            {
                Left = 16,
                Top = 188,
                Width = 200,
                Height = 24,
                DropDownStyle = ComboBoxStyle.DropDownList
            };
            bodyConfigComboBox.SelectedIndexChanged += (_, _) => RefreshBodyEditorValues();

            capturePoseButton = new Button
            {
                Left = 224,
                Top = 186,
                Width = 80,
                Height = 28,
                Text = "Capture"
            };
            capturePoseButton.Click += (_, _) => CaptureBodyEditorValues();

            armTriggerButton = new Button
            {
                Left = 312,
                Top = 186,
                Width = 92,
                Height = 28,
                Text = "Arm Trigger"
            };
            armTriggerButton.Click += (_, _) => ArmTriggerCaptureFromEditor();

            applyCheckedButton = new Button
            {
                Left = 412,
                Top = 186,
                Width = 88,
                Height = 28,
                Text = "Apply"
            };
            applyCheckedButton.Click += (_, _) => ApplyEditorToCheckedTrackers();

            saveConfigButton = new Button
            {
                Left = 508,
                Top = 186,
                Width = 80,
                Height = 28,
                Text = "Save"
            };
            saveConfigButton.Click += (_, _) => SaveBodyEditorValues();

            var batchLabel = new Label
            {
                Left = 16,
                Top = 224,
                Width = 260,
                Height = 20,
                Text = "Checked trackers"
            };

            selectAllTrackersButton = new Button
            {
                Left = 420,
                Top = 220,
                Width = 80,
                Height = 28,
                Text = "All"
            };
            selectAllTrackersButton.Click += (_, _) => SetAllBatchTrackersChecked(true);

            selectNoTrackersButton = new Button
            {
                Left = 508,
                Top = 220,
                Width = 80,
                Height = 28,
                Text = "None"
            };
            selectNoTrackersButton.Click += (_, _) => SetAllBatchTrackersChecked(false);

            batchTrackerListBox = new CheckedListBox
            {
                Left = 16,
                Top = 252,
                Width = 572,
                Height = 78,
                CheckOnClick = true
            };

            saveStatusLabel = new Label
            {
                Left = 16,
                Top = 340,
                Width = 572,
                Height = 20,
                TextAlign = System.Drawing.ContentAlignment.MiddleLeft
            };

            var positionLabel = new Label
            {
                Left = 16,
                Top = 370,
                Width = 100,
                Height = 20,
                Text = "Position m"
            };

            var positionXLabel = new Label { Left = 124, Top = 370, Width = 32, Height = 20, Text = "X" };
            var positionYLabel = new Label { Left = 272, Top = 370, Width = 32, Height = 20, Text = "Y" };
            var positionZLabel = new Label { Left = 420, Top = 370, Width = 32, Height = 20, Text = "Z" };

            positionXInput = CreateNumberInput(124, 392, -10m, 10m, 0.001m, 6);
            positionYInput = CreateNumberInput(272, 392, -10m, 10m, 0.001m, 6);
            positionZInput = CreateNumberInput(420, 392, -10m, 10m, 0.001m, 6);

            var rotationLabel = new Label
            {
                Left = 16,
                Top = 426,
                Width = 100,
                Height = 20,
                Text = "Rotation deg"
            };

            var rotationXLabel = new Label { Left = 124, Top = 426, Width = 48, Height = 20, Text = "Pitch" };
            var rotationYLabel = new Label { Left = 272, Top = 426, Width = 48, Height = 20, Text = "Yaw" };
            var rotationZLabel = new Label { Left = 420, Top = 426, Width = 48, Height = 20, Text = "Roll" };

            rotationXInput = CreateNumberInput(124, 448, -360m, 360m, 0.1m, 3);
            rotationYInput = CreateNumberInput(272, 448, -360m, 360m, 0.1m, 3);
            rotationZInput = CreateNumberInput(420, 448, -360m, 360m, 0.1m, 3);

            var rollOffsetLabel = new Label
            {
                Left = 16,
                Top = 484,
                Width = 160,
                Height = 20,
                Text = "Roll offset deg"
            };

            rollOffsetInput = CreateNumberInput(124, 506, -360m, 360m, 0.1m, 3);

            positionXInput.ValueChanged += (_, _) => ApplyBodyEditorValues();
            positionYInput.ValueChanged += (_, _) => ApplyBodyEditorValues();
            positionZInput.ValueChanged += (_, _) => ApplyBodyEditorValues();
            rotationXInput.ValueChanged += (_, _) => ApplyBodyEditorValues();
            rotationYInput.ValueChanged += (_, _) => ApplyBodyEditorValues();
            rotationZInput.ValueChanged += (_, _) => ApplyBodyEditorValues();
            rollOffsetInput.ValueChanged += (_, _) => ApplyBodyEditorValues();

            var positionPredictionLabel = new Label
            {
                Left = 16,
                Top = 544,
                Width = 220,
                Height = 20,
                Text = "Position prediction"
            };

            positionPredictionValueLabel = new Label
            {
                Left = 456,
                Top = 544,
                Width = 132,
                Height = 20,
                TextAlign = System.Drawing.ContentAlignment.MiddleRight
            };

            positionPredictionTrackBar = new TrackBar
            {
                Left = 16,
                Top = 564,
                Width = 572,
                Height = 44,
                Minimum = 0,
                Maximum = 100,
                TickFrequency = 10,
                SmallChange = 1,
                LargeChange = 5
            };
            positionPredictionTrackBar.ValueChanged += (_, _) =>
            {
                if (refreshingControls) return;
                setPositionPredictionStrength(positionPredictionTrackBar.Value / 100f);
                RefreshState();
            };

            statusLabel = new Label
            {
                Left = 16,
                Top = 626,
                Width = 572,
                Height = 24,
                TextAlign = System.Drawing.ContentAlignment.MiddleCenter
            };

            toggleButton = new Button
            {
                Left = 16,
                Top = 654,
                Width = 572,
                Height = 44,
                UseVisualStyleBackColor = false
            };
            toggleButton.Click += (_, _) => ToggleCalibration();

            Controls.Add(presetLabel);
            Controls.Add(presetComboBox);
            Controls.Add(refreshPresetButton);
            Controls.Add(applyPresetButton);
            Controls.Add(selectedPathTextBox);
            Controls.Add(currentLabel);
            Controls.Add(currentPathTextBox);
            Controls.Add(bodyLabel);
            Controls.Add(bodyConfigComboBox);
            Controls.Add(capturePoseButton);
            Controls.Add(armTriggerButton);
            Controls.Add(applyCheckedButton);
            Controls.Add(saveConfigButton);
            Controls.Add(batchLabel);
            Controls.Add(selectAllTrackersButton);
            Controls.Add(selectNoTrackersButton);
            Controls.Add(batchTrackerListBox);
            Controls.Add(saveStatusLabel);
            Controls.Add(positionLabel);
            Controls.Add(positionXLabel);
            Controls.Add(positionYLabel);
            Controls.Add(positionZLabel);
            Controls.Add(positionXInput);
            Controls.Add(positionYInput);
            Controls.Add(positionZInput);
            Controls.Add(rotationLabel);
            Controls.Add(rotationXLabel);
            Controls.Add(rotationYLabel);
            Controls.Add(rotationZLabel);
            Controls.Add(rotationXInput);
            Controls.Add(rotationYInput);
            Controls.Add(rotationZInput);
            Controls.Add(rollOffsetLabel);
            Controls.Add(rollOffsetInput);
            Controls.Add(positionPredictionLabel);
            Controls.Add(positionPredictionValueLabel);
            Controls.Add(positionPredictionTrackBar);
            Controls.Add(statusLabel);
            Controls.Add(toggleButton);

            refreshTimer = new System.Windows.Forms.Timer { Interval = 100 };
            refreshTimer.Tick += (_, _) => RefreshState();
            refreshTimer.Start();

            FormClosed += (_, _) =>
            {
                refreshTimer.Stop();
                requestExit();
            };

            LoadPresetChoices();
            LoadBodyChoices();
            RefreshState();
        }

        static NumericUpDown CreateNumberInput(int left, int top, decimal minimum, decimal maximum, decimal increment, int decimalPlaces)
        {
            return new NumericUpDown
            {
                Left = left,
                Top = top,
                Width = 120,
                Height = 24,
                Minimum = minimum,
                Maximum = maximum,
                Increment = increment,
                DecimalPlaces = decimalPlaces
            };
        }

        void LoadPresetChoices()
        {
            string currentPath = getCurrentConfigPath();
            var choices = getPresetChoices();

            presetComboBox.Items.Clear();
            presetComboBox.Items.AddRange(choices.Cast<object>().ToArray());

            if (choices.Count == 0)
            {
                selectedPathTextBox.Text = string.Empty;
                applyPresetButton.Enabled = false;
                return;
            }

            int selectedIndex = choices.FindIndex(choice =>
                string.Equals(Path.GetFullPath(choice.ConfigPath), Path.GetFullPath(currentPath), StringComparison.OrdinalIgnoreCase));

            presetComboBox.SelectedIndex = selectedIndex >= 0 ? selectedIndex : 0;
            applyPresetButton.Enabled = true;
            RefreshSelectedPath();
        }

        void RefreshSelectedPath()
        {
            if (presetComboBox.SelectedItem is PresetChoice choice)
            {
                selectedPathTextBox.Text = choice.ConfigPath;
            }
        }

        void ApplySelectedPreset()
        {
            if (presetComboBox.SelectedItem is not PresetChoice choice)
            {
                return;
            }

            requestConfigReload(choice.ConfigPath);
        }

        void LoadBodyChoices()
        {
            string currentPath = getCurrentConfigPath();
            var configs = getBodyConfigs();
            string previousSelection = bodyConfigComboBox.SelectedItem is BodyConfigChoice previous
                ? previous.Config.VmtId.ToString(CultureInfo.InvariantCulture)
                : string.Empty;
            var previousCheckedIds = GetCheckedBatchVmtIds();

            refreshingControls = true;
            bodyConfigComboBox.Items.Clear();
            bodyConfigComboBox.Items.AddRange(configs.Select(config => new BodyConfigChoice(config)).Cast<object>().ToArray());
            batchTrackerListBox.Items.Clear();
            batchTrackerListBox.Items.AddRange(configs.Select(config => new BodyConfigChoice(config)).Cast<object>().ToArray());
            lastBodyConfigPath = currentPath;
            lastBodyConfigCount = configs.Count;

            if (configs.Count == 0)
            {
                SetBodyEditorEnabled(false);
                saveStatusLabel.Text = string.Empty;
                refreshingControls = false;
                return;
            }

            int selectedIndex = 0;
            if (previousSelection.Length > 0)
            {
                for (int i = 0; i < bodyConfigComboBox.Items.Count; i++)
                {
                    if (bodyConfigComboBox.Items[i] is BodyConfigChoice choice
                        && choice.Config.VmtId.ToString(CultureInfo.InvariantCulture) == previousSelection)
                    {
                        selectedIndex = i;
                        break;
                    }
                }
            }

            bodyConfigComboBox.SelectedIndex = selectedIndex;
            if (previousCheckedIds.Count > 0)
            {
                for (int i = 0; i < batchTrackerListBox.Items.Count; i++)
                {
                    if (batchTrackerListBox.Items[i] is BodyConfigChoice choice
                        && previousCheckedIds.Contains(choice.Config.VmtId))
                    {
                        batchTrackerListBox.SetItemChecked(i, true);
                    }
                }
            }
            else if (selectedIndex >= 0 && selectedIndex < batchTrackerListBox.Items.Count)
            {
                batchTrackerListBox.SetItemChecked(selectedIndex, true);
            }

            SetBodyEditorEnabled(true);
            refreshingControls = false;
            RefreshBodyEditorValues();
        }

        void RefreshBodyChoicesIfNeeded()
        {
            string currentPath = getCurrentConfigPath();
            int configCount = getBodyConfigs().Count;
            if (!string.Equals(currentPath, lastBodyConfigPath, StringComparison.OrdinalIgnoreCase)
                || configCount != lastBodyConfigCount)
            {
                LoadBodyChoices();
            }
        }

        void RefreshBodyEditorValues()
        {
            if (refreshingControls) return;

            if (bodyConfigComboBox.SelectedItem is not BodyConfigChoice choice)
            {
                SetBodyEditorEnabled(false);
                return;
            }

            refreshingControls = true;
            SetNumberInput(positionXInput, choice.Config.PositionOffset.X);
            SetNumberInput(positionYInput, choice.Config.PositionOffset.Y);
            SetNumberInput(positionZInput, choice.Config.PositionOffset.Z);
            SetNumberInput(rotationXInput, choice.Config.RotationOffset.X);
            SetNumberInput(rotationYInput, choice.Config.RotationOffset.Y);
            SetNumberInput(rotationZInput, choice.Config.RotationOffset.Z);
            SetNumberInput(rollOffsetInput, choice.Config.RollOffset);
            SetBodyEditorEnabled(true);
            refreshingControls = false;
        }

        void ApplyBodyEditorValues()
        {
            if (refreshingControls) return;
            if (bodyConfigComboBox.SelectedItem is not BodyConfigChoice choice) return;

            choice.Config.PositionOffset = new Vector3(
                (float)positionXInput.Value,
                (float)positionYInput.Value,
                (float)positionZInput.Value);
            choice.Config.RotationOffset = new Vector3(
                (float)rotationXInput.Value,
                (float)rotationYInput.Value,
                (float)rotationZInput.Value);
            choice.Config.RollOffset = (float)rollOffsetInput.Value;
            saveStatusLabel.Text = "Edited";
        }

        void CaptureBodyEditorValues()
        {
            saveStatusLabel.Text = captureCurrentPoses(GetCheckedBodyConfigs());
            RefreshBodyEditorValues();
        }

        void ArmTriggerCaptureFromEditor()
        {
            saveStatusLabel.Text = armTriggerCapture(GetCheckedBodyConfigs());
            lastTriggerStatus = saveStatusLabel.Text;
        }

        void ApplyEditorToCheckedTrackers()
        {
            var configs = GetCheckedBodyConfigs();
            if (configs.Count == 0)
            {
                saveStatusLabel.Text = "Apply failed: no trackers checked.";
                return;
            }

            Vector3 position = new(
                (float)positionXInput.Value,
                (float)positionYInput.Value,
                (float)positionZInput.Value);
            Vector3 rotation = new(
                (float)rotationXInput.Value,
                (float)rotationYInput.Value,
                (float)rotationZInput.Value);
            float rollOffset = (float)rollOffsetInput.Value;

            foreach (var config in configs)
            {
                config.PositionOffset = position;
                config.RotationOffset = rotation;
                config.RollOffset = rollOffset;
            }

            saveStatusLabel.Text = configs.Count == 1
                ? $"Applied: {configs[0].Name}"
                : $"Applied: {configs.Count} trackers";
            RefreshBodyEditorValues();
        }

        void SaveBodyEditorValues()
        {
            ApplyBodyEditorValues();
            saveStatusLabel.Text = saveCurrentConfig();
        }

        void SetBodyEditorEnabled(bool enabled)
        {
            bodyConfigComboBox.Enabled = enabled;
            positionXInput.Enabled = enabled;
            positionYInput.Enabled = enabled;
            positionZInput.Enabled = enabled;
            rotationXInput.Enabled = enabled;
            rotationYInput.Enabled = enabled;
            rotationZInput.Enabled = enabled;
            rollOffsetInput.Enabled = enabled;
            capturePoseButton.Enabled = enabled;
            armTriggerButton.Enabled = enabled;
            applyCheckedButton.Enabled = enabled;
            saveConfigButton.Enabled = enabled;
            batchTrackerListBox.Enabled = enabled;
            selectAllTrackersButton.Enabled = enabled;
            selectNoTrackersButton.Enabled = enabled;
        }

        List<BodyPartConfig> GetCheckedBodyConfigs()
        {
            return batchTrackerListBox.CheckedItems
                .OfType<BodyConfigChoice>()
                .Select(choice => choice.Config)
                .ToList();
        }

        HashSet<int> GetCheckedBatchVmtIds()
        {
            return batchTrackerListBox.CheckedItems
                .OfType<BodyConfigChoice>()
                .Select(choice => choice.Config.VmtId)
                .ToHashSet();
        }

        void SetAllBatchTrackersChecked(bool isChecked)
        {
            for (int i = 0; i < batchTrackerListBox.Items.Count; i++)
            {
                batchTrackerListBox.SetItemChecked(i, isChecked);
            }
        }

        static void SetNumberInput(NumericUpDown input, float value)
        {
            decimal decimalValue = (decimal)value;
            if (decimalValue < input.Minimum) decimalValue = input.Minimum;
            if (decimalValue > input.Maximum) decimalValue = input.Maximum;
            input.Value = decimalValue;
        }

        void ToggleCalibration()
        {
            setCalibrationActive(!getCalibrationActive());
            RefreshState();
        }

        void RefreshState()
        {
            RefreshBodyChoicesIfNeeded();

            string triggerStatus = getTriggerCaptureStatus();
            if (triggerStatus.Length > 0 && triggerStatus != lastTriggerStatus)
            {
                lastTriggerStatus = triggerStatus;
                saveStatusLabel.Text = triggerStatus;
                if (triggerStatus.StartsWith("Captured:", StringComparison.Ordinal))
                {
                    RefreshBodyEditorValues();
                }
            }

            refreshingControls = true;
            int predictionPercent = Math.Clamp((int)MathF.Round(getPositionPredictionStrength() * 100f), 0, 100);
            if (positionPredictionTrackBar.Value != predictionPercent)
            {
                positionPredictionTrackBar.Value = predictionPercent;
            }

            positionPredictionValueLabel.Text = $"{predictionPercent}% ({getEffectivePositionPredictionMs():F1} ms)";
            refreshingControls = false;

            bool active = getCalibrationActive();
            currentPathTextBox.Text = getCurrentConfigPath();
            statusLabel.Text = active ? "Calibration locked" : "Live tracking";
            toggleButton.Text = active ? "Unlock Calibration" : "Lock Calibration";
            toggleButton.BackColor = active ? System.Drawing.Color.FromArgb(245, 210, 95) : System.Drawing.Color.FromArgb(125, 215, 150);
        }
    }

    internal static class NativeMethods
    {
        [System.Runtime.InteropServices.DllImport("winmm.dll", EntryPoint = "timeBeginPeriod")]
        public static extern uint TimeBeginPeriod(uint period);

        [System.Runtime.InteropServices.DllImport("winmm.dll", EntryPoint = "timeEndPeriod")]
        public static extern uint TimeEndPeriod(uint period);

        [System.Runtime.InteropServices.DllImport("user32.dll")]
        public static extern short GetAsyncKeyState(int vKey);
        public static bool IsKeyDown(int vKey) => (GetAsyncKeyState(vKey) & 0x8000) != 0;
    }
}
