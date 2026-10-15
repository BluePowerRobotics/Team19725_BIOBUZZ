function analyze_tuning(varargin)
%ANALYZE_TUNING 飞轮标定数据离线分析 / 参数复算（MATLAB 对照版）
%
% 读取 RC 上 OpModes/Tuning 五个标定 OpMode 导出的 CSV，自动判别方法、出图，
% 并用与机器人端**同一套公式**复算控制参数。与 tools/tuning/analyze_tuning.py
% 输出口径一致，用于互相印证（例如与既有 MATLAB 工作区成果对齐）。
%
% 对应《飞轮系统控制理论研究报告》第 6 章：
%   6.1 step01  StepResponseTuner     -> Kd / tau / kff / kp / ki
%   6.2 zn02    ZieglerNicholsTuner   -> Ku / Tu / kp / ki
%   6.3 relay03 RelayFeedbackTuner    -> a / Tu / Ku / kp / ki
%   6.4 drop04  DropDetectionTuner    -> sigma / 量化台阶 / maxDecel / 阈值 / kBoost
%   6.5 ff05    FeedforwardTuner      -> 多点 (omega, d_ss) 回归 Kff
%
% 用法：
%   analyze_tuning('C:\path\to\tuning')                  % 目录下全部 csv
%   analyze_tuning('step01_1.csv','zn02_1.csv')          % 指定若干文件
%   analyze_tuning(DIR,'-out',OUTDIR,'-noplot')          % 只出报告不出图
%   analyze_tuning(F,'-window',4,'-ku',9e-5,'-kp',5e-5)  % 覆盖分析参数
%
% 选项：-out DIR | -noplot | -window S | -ku K | -kp K | -alpha A
%
% CSV 格式（TuningLogger 生成）：'#' 开头为注释行（含本次实验参数与结论，形如 key=value），
% 随后表头 t,duty,velocity 与数据行；时间单位 s，转速单位 tick/s。
% 注意：step01 在阶跃处插入 '# --- step applied here ... ---'，其后 t 从 0 重新开始。

clc;
cfg = struct('out', '', 'plot', true, 'window', [], 'ku', [], 'kp', [], 'alpha', 0.3);
inputs = {};
i = 1;
while i <= numel(varargin)
    a = varargin{i};
    if ischar(a) && ~isempty(a) && a(1) == '-'
        switch lower(a)
            case '-out',    cfg.out = varargin{i+1}; i = i + 2;
            case '-noplot', cfg.plot = false;        i = i + 1;
            case '-window', cfg.window = varargin{i+1}; i = i + 2;
            case '-ku',     cfg.ku = varargin{i+1};  i = i + 2;
            case '-kp',     cfg.kp = varargin{i+1};  i = i + 2;
            case '-alpha',  cfg.alpha = varargin{i+1}; i = i + 2;
            otherwise, error('未知选项：%s', a);
        end
    else
        inputs{end+1} = a; %#ok<AGROW>
        i = i + 1;
    end
end
if isempty(inputs)
    error('请给出 CSV 文件或目录');
end

files = {};
for k = 1:numel(inputs)
    p = inputs{k};
    if isfolder(p)
        f = dir(fullfile(p, '*.csv'));
        for j = 1:numel(f)
            files{end+1} = fullfile(f(j).folder, f(j).name); %#ok<AGROW>
        end
    else
        files{end+1} = p; %#ok<AGROW>
    end
end

for k = 1:numel(files)
    path = files{k};
    if ~isfile(path)
        fprintf('[跳过] 不存在：%s\n', path);
        continue;
    end
    d = readTuningCsv(path);
    [res, method] = analyzeOne(d, cfg);
    txt = buildReport(d, res, method);
    fprintf('%s\n', txt);

    if isempty(cfg.out)
        outDir = fileparts(path);
    else
        outDir = cfg.out;
        if ~isfolder(outDir), mkdir(outDir); end
    end
    [~, stem] = fileparts(path);
    writeUtf8(fullfile(outDir, [stem '_report.txt']), txt);
    if cfg.plot
        try
            makePlots(d, res, method, fullfile(outDir, [stem '_analysis.png']));
            fprintf('[图] %s\n', fullfile(outDir, [stem '_analysis.png']));
        catch ME
            fprintf('[绘图失败] %s: %s\n', ME.identifier, ME.message);
        end
    end
end
end

% =========================================================================
% CSV 读取
% =========================================================================

function d = readTuningCsv(path)
d = struct('path', path, 'meta', {{}}, 'events', {{}}, 'header', {{}}, ...
           't', [], 'duty', [], 'vel', []);
fid = fopen(path, 'r', 'n', 'UTF-8');
if fid < 0
    error('无法打开 %s', path);
end
ts = []; ds = []; vs = [];
nrow = 0;
while true
    line = fgetl(fid);
    if ~ischar(line), break; end
    line = strtrim(line);
    if isempty(line), continue; end
    if line(1) == '#'
        text = strtrim(line(2:end));
        d.meta{end+1} = text; %#ok<AGROW>
        d.events{end+1} = {nrow, text}; %#ok<AGROW>
        continue;
    end
    parts = strsplit(line, ',');
    if numel(parts) < 3, continue; end
    v1 = str2double(parts{1});
    v2 = str2double(parts{2});
    v3 = str2double(parts{3});
    if any(isnan([v1 v2 v3]))
        d.header = parts;   % 表头行（非数字）
        continue;
    end
    ts(end+1, 1) = v1; %#ok<AGROW>
    ds(end+1, 1) = v2; %#ok<AGROW>
    vs(end+1, 1) = v3; %#ok<AGROW>
    nrow = nrow + 1;
end
fclose(fid);
d.t = ts; d.duty = ds; d.vel = vs;
end

function v = metaGet(meta, key, def)
% 从注释行里取 `key=value`（数字）。大小写敏感，key 后必须紧跟 '='。
v = def;
pat = ['(^|[^A-Za-z0-9_])' regexptranslate('escape', key) '\s*=\s*(-?[0-9]*\.?[0-9]+([eE][-+]?[0-9]+)?)'];
for k = 1:numel(meta)
    tok = regexp(meta{k}, pat, 'tokens', 'once');
    if ~isempty(tok)
        v = str2double(tok{2});
        return;
    end
end
end

% =========================================================================
% 通用工具（与机器人端同一口径）
% =========================================================================

function g = gcdStep(v)
g = 0;
for k = 1:numel(v)
    iv = round(abs(v(k)));
    if iv > 0
        g = gcd(g, iv);
    end
end
g = double(g);
end

function m = emaMaxDecel(t, v, alpha)
m = 0;
if numel(t) < 2, return; end
ema = v(1); prevEma = ema; prevT = t(1);
for k = 2:numel(t)
    ema = ema + alpha * (v(k) - ema);
    dt = t(k) - prevT;
    dv = ema - prevEma;
    if dt > 1e-4 && dv < 0
        decel = -dv / dt;
        if decel > m, m = decel; end
    end
    prevEma = ema; prevT = t(k);
end
end

function [Tu, amp, meanV] = oscPeriod(t, v, minCrossings)
if nargin < 3, minCrossings = 4; end
Tu = NaN; amp = 0; meanV = 0;
if numel(t) < 10, return; end
meanV = mean(v);
amp = (max(v) - min(v)) / 2;
firstT = NaN; lastT = NaN; crossings = 0;
for k = 2:numel(v)
    a = v(k-1) - meanV; b = v(k) - meanV;
    if (a < 0 && b >= 0) || (a > 0 && b <= 0)
        if isnan(firstT), firstT = t(k); end
        lastT = t(k);
        crossings = crossings + 1;
    end
end
if crossings < minCrossings || isnan(firstT) || lastT <= firstT, return; end
cycles = floor((crossings - 1) / 2);
if cycles < 1, return; end
Tu = (lastT - firstT) / cycles;
end

function fit = firstOrderFit(t, v)
% 最小二乘拟合 v(t) = v0 + K*(1-exp(-t/tau))：对 tau 做黄金分割，其余参数线性求解
fit = [];
if numel(t) < 5, return; end
f = @(tau) sseOf(tau, t, v);
gr = (sqrt(5) - 1) / 2;
lo = 1e-3; hi = 20;
c = hi - gr * (hi - lo); dd = lo + gr * (hi - lo);
for it = 1:200
    if f(c) < f(dd), hi = dd; else, lo = c; end
    c = hi - gr * (hi - lo); dd = lo + gr * (hi - lo);
end
tau = 0.5 * (lo + hi);
x = 1 - exp(-t / tau);
A = [ones(numel(t), 1), x];
coef = A \ v;
fit = struct('tau', tau, 'v0', coef(1), 'K', coef(2), 'sse', sum((v - A * coef).^2));
end

function s = sseOf(tau, t, v)
x = 1 - exp(-t / tau);
A = [ones(numel(t), 1), x];
coef = A \ v;
r = v - A * coef;
s = r' * r;
end

% =========================================================================
% 各方法分析
% =========================================================================

function [res, method] = analyzeOne(d, cfg)
method = detectMethod(d);
res = struct('ok', false, 'msg', '');
switch method
    case 'step01',  res = analyzeStep(d, cfg);
    case 'zn02',    res = analyzeZn(d, cfg);
    case 'relay03', res = analyzeRelay(d, cfg);
    case 'drop04',  res = analyzeDrop(d, cfg);
    case 'ff05',    res = analyzeFf(d, cfg);
    otherwise
        res.ok = false;
        res.msg = '未识别方法：仅输出基本统计';
end
end

function m = detectMethod(d)
[~, base] = fileparts(d.path);
base = lower(base);
keys = {'step01', 'zn02', 'relay03', 'drop04', 'ff05'};
m = 'unknown';
for k = 1:numel(keys)
    if startsWith(base, keys{k})
        m = keys{k};
        return;
    end
end
blob = lower(strjoin(d.meta, ' '));
if contains(blob, 'step response'),      m = 'step01';
elseif contains(blob, 'critical-gain'),  m = 'zn02';
elseif contains(blob, 'relay feedback'), m = 'relay03';
elseif contains(blob, 'drop-detection'), m = 'drop04';
elseif contains(blob, 'feedforward'),    m = 'ff05';
end
end

function res = analyzeStep(d, cfg) %#ok<INUSD>
res = struct('ok', false);
idx = 0;
for k = 1:numel(d.events)
    if contains(d.events{k}{2}, 'step applied here')
        idx = d.events{k}{1};
        break;
    end
end
preT = d.t(1:idx);            preV = d.vel(1:idx);   preD = d.duty(1:idx);
postT = d.t(idx+1:end);       postV = d.vel(idx+1:end); postD = d.duty(idx+1:end);
if numel(postT) < 5
    res.msg = '阶跃段采样不足（<5 点）';
    return;
end
win = metaGet(d.meta, 'steadyWindowSec', 0.5);
if isempty(preD), d0 = metaGet(d.meta, 'd0', 0); else, d0 = median(preD); end
if isempty(postD), dStep = metaGet(d.meta, 'dStep', 1); else, dStep = median(postD); end
omega0 = tailMean(preT, preV, win);
omegaSS = tailMean(postT, postV, win);
deltaD = dStep - d0; deltaW = omegaSS - omega0;
if abs(deltaD) < 1e-9 || abs(deltaW) < 1e-9
    res.msg = 'Δd 或 Δω 过小：dStep 与 d0 太接近，或飞轮没真正转起来';
    res.d0 = d0; res.dStep = dStep; res.omega0 = omega0; res.omegaSS = omegaSS;
    return;
end
kd = deltaW / deltaD;
tau632 = NaN;
target = omega0 + 0.632 * deltaW;
hit = find(postV >= target, 1, 'first');
if ~isempty(hit), tau632 = postT(hit); end
fit = firstOrderFit(postT, postV);
tRecover = metaGet(d.meta, 'tRecover', 0.5);
ratio = metaGet(d.meta, 'ratio', 4.0);
tauC = tRecover / ratio;
res = struct('ok', true, 'd0', d0, 'd_step', dStep, 'omega0', omega0, 'omega_ss', omegaSS, ...
             'kd', kd, 'tau_632', tau632, 'fit', fit, 'tau_c', tauC, ...
             'sug_632', suggestPI(tau632, kd, tauC), 'sug_ls', [], 'msg', '');
if ~isempty(fit), res.sug_ls = suggestPI(fit.tau, kd, tauC); end
end

function tm = tailMean(t, v, win)
if isempty(t), tm = 0; return; end
sel = t >= (t(end) - win);
if any(sel), tm = mean(v(sel)); else, tm = mean(v); end
end

function s = suggestPI(tau, kd, tauC)
if isempty(tau) || isnan(tau) || kd <= 0 || tauC <= 0
    s = [];
    return;
end
s = struct('tau', tau, 'kff', 1 / kd, 'kp', tau / (kd * tauC), 'ki', 1 / (kd * tauC));
end

function res = analyzeZn(d, cfg)
if isempty(cfg.window), win = 5.0; else, win = cfg.window; end
sel = d.t >= (d.t(end) - win);
[Tu, amp, meanV] = oscPeriod(d.t(sel), d.vel(sel));
ku = metaGet(d.meta, 'Ku', []);
if isempty(ku) && ~isempty(cfg.ku), ku = cfg.ku; end
res = struct('ok', true, 'amp', amp, 'mean', meanV, 'tu', Tu, 'ku', ku, ...
             'tu_locked', metaGet(d.meta, 'Tu', []), 'window', win, 'msg', '');
if isempty(ku)
    res.msg = 'CSV 里没有锁定的 Ku——请在 OpMode 里按 A 锁定后重跑，或用 -ku 指定';
    return;
end
if ~isnan(Tu)
    res.kp = 0.45 * ku;
    res.ki = 0.54 * ku / Tu;
end
end

function res = analyzeRelay(d, cfg)
measure = metaGet(d.meta, 'measureSec', []);
if isempty(cfg.window)
    if isempty(measure), win = 4.0; else, win = measure; end
else
    win = cfg.window;
end
sel = d.t >= (d.t(end) - win);
[Tu, amp, meanV] = oscPeriod(d.t(sel), d.vel(sel));
dHigh = max(d.duty(sel)); dLow = min(d.duty(sel));
deltaD = dHigh - dLow;
res = struct('ok', true, 'amp', amp, 'mean', meanV, 'tu', Tu, 'd_high', dHigh, ...
             'd_low', dLow, 'delta_d', deltaD, 'window', win, 'msg', '', 'ku', []);
if amp > 1e-9
    res.ku = 4 * deltaD / (pi * amp);
    if ~isnan(Tu)
        res.kp = 0.45 * res.ku;
        res.ki = 0.54 * res.ku / Tu;
    end
end
end

function res = analyzeDrop(d, cfg)
measure = metaGet(d.meta, 'measureSec', 5.0);
sel = d.t >= (d.t(end) - measure);
t = d.t(sel); v = d.vel(sel);
if numel(v) < 10
    res = struct('ok', false, 'msg', '测量段采样不足（<10 点）');
    return;
end
meanV = mean(v);
sd = std(v, 0);                    % 样本标准差 ÷N-1（MATLAB std 默认 N-1）
step = gcdStep(v);
maxDecel = emaMaxDecel(t, v, cfg.alpha);
kSigma = metaGet(d.meta, 'kSigma', 5.0);
kRate = metaGet(d.meta, 'kRate', 2.5);
kRatio = metaGet(d.meta, 'kBoostRatio', 2.0);
dropTh = kSigma * sd;
dropRate = kRate * maxDecel;
kBoostCsv = metaGet(d.meta, 'kBoost', []);
kp = metaGet(d.meta, 'kp', []);
if isempty(kp), kp = cfg.kp; end
if ~isempty(kBoostCsv)
    kBoost = kBoostCsv;
elseif ~isempty(kp)
    kBoost = kRatio * kp;
else
    kBoost = [];
end
res = struct('ok', true, 'mean', meanV, 'sd', sd, 'step', step, 'max_decel', maxDecel, ...
             'alpha', cfg.alpha, 'k_sigma', kSigma, 'k_rate', kRate, 'k_ratio', kRatio, ...
             'drop_threshold', dropTh, 'drop_rate_threshold', dropRate, 'k_boost', kBoost, ...
             'sigma_steps', sd / max(step, eps), 'threshold_steps', dropTh / max(step, eps), ...
             'onboard', struct('sigma', metaGet(d.meta, 'sigma', []), 'step', metaGet(d.meta, 'step', []), ...
                               'max_decel', metaGet(d.meta, 'maxDecel', []), ...
                               'drop_threshold', metaGet(d.meta, 'dropThreshold', []), ...
                               'drop_rate_threshold', metaGet(d.meta, 'dropRate', []), ...
                               'k_boost', kBoostCsv), 'msg', '');
end

function res = analyzeFf(d, cfg) %#ok<INUSD>
w = []; ds = [];
for k = 1:numel(d.meta)
    if startsWith(d.meta{k}, 'point')
        a = metaGet(d.meta(k), 'omega', []);
        b = metaGet(d.meta(k), 'dss', []);
        if ~isempty(a) && ~isempty(b)
            w(end+1, 1) = a; %#ok<AGROW>
            ds(end+1, 1) = b; %#ok<AGROW>
        end
    end
end
res = struct('ok', ~isempty(w), 'points', [w, ds], 'msg', '');
if isempty(w)
    res.msg = 'CSV 里没有 `# point ...` 记录——请在 OpMode 里按 A 逐点记录后重跑';
    return;
end
res.kff_origin = sum(w .* ds) / sum(w .* w);
if numel(w) >= 2
    p = polyfit(w, ds, 1);
    res.slope = p(1);
    res.intercept = p(2);
end
end

% =========================================================================
% 报告
% =========================================================================

function txt = buildReport(d, res, method)
L = {};
[span, rate] = spanAndRate(d.t);
L{end+1} = repmat('=', 1, 78);
L{end+1} = sprintf('文件：%s', d.path);
L{end+1} = sprintf('方法：%s    采样点：%d    时长：%.2f s    行率：%.1f Hz', method, numel(d.t), span, rate);
L{end+1} = repmat('=', 1, 78);

switch method
    case 'step01'
        if ~res.ok
            L{end+1} = sprintf('[无效] %s', res.msg);
            txt = strjoin(L, newline); return;
        end
        L{end+1} = sprintf('d0 = %.4f    dStep = %.4f    Δd = %.4f', res.d0, res.d_step, res.d_step - res.d0);
        L{end+1} = sprintf('ω0 = %.1f    ω(∞) = %.1f    Δω = %.1f tick/s', res.omega0, res.omega_ss, res.omega_ss - res.omega0);
        L{end+1} = sprintf('Kd = Δω/Δd = %.4f (tick/s)/占空比', res.kd);
        if isnan(res.tau_632)
            L{end+1} = 'τ (t63.2%) = n/a';
        else
            L{end+1} = sprintf('τ (t63.2%%) = %.4f s', res.tau_632);
        end
        if ~isempty(res.fit)
            L{end+1} = sprintf('τ (最小二乘) = %.4f s   (拟合残差 SSE=%.3g)', res.fit.tau, res.fit.sse);
        end
        L{end+1} = sprintf('τc = tRecover/ratio = %.4f s', res.tau_c);
        if ~isempty(res.sug_632)
            L{end+1} = sprintf('  建议参数（按 t63.2%%）：kff = %.8f   kp = %.8f   ki = %.8f', ...
                               res.sug_632.kff, res.sug_632.kp, res.sug_632.ki);
        end
        if ~isempty(res.sug_ls)
            L{end+1} = sprintf('  建议参数（按最小二乘）：kff = %.8f   kp = %.8f   ki = %.8f', ...
                               res.sug_ls.kff, res.sug_ls.kp, res.sug_ls.ki);
        end
        L{end+1} = '  → 填入 PrototypeShooter：kff / kp / ki';
        L{end+1} = '  提示：kff 也可用方法 6.5 的实测稳态占空比修正；Kd 偏小时说明 Δd 太小，加大阶跃幅度重跑。';

    case 'zn02'
        L{end+1} = sprintf('分析窗口：尾部 %.2f s', res.window);
        if isnan(res.tu)
            tuStr = 'n/a（过零不足或幅值太小）';
        else
            tuStr = sprintf('%.4f s', res.tu);
        end
        L{end+1} = sprintf('等幅振荡估计：幅值 a = %.1f tick/s    中心 = %.1f    Tu = %s', res.amp, res.mean, tuStr);
        if isempty(res.ku)
            L{end+1} = sprintf('[无效] %s', res.msg);
            txt = strjoin(L, newline); return;
        end
        if isempty(metaGet(d.meta, 'Ku', []))
            src = '命令行 -ku';
        else
            src = 'lock (A 键锁定)';
        end
        L{end+1} = sprintf('Ku = %.8f   （来源：%s）', res.ku, src);
        if ~isempty(res.tu_locked)
            L{end+1} = sprintf('锁定时的 Tu = %.4f s', res.tu_locked);
        end
        if isfield(res, 'kp')
            L{end+1} = sprintf('ZN 表：建议 kp = 0.45·Ku = %.8f    ki = 0.54·Ku/Tu = %.8f', res.kp, res.ki);
        end

    case 'relay03'
        L{end+1} = sprintf('分析窗口：尾部 %.2f s', res.window);
        L{end+1} = sprintf('继电器输出：dHigh = %.4f   dLow = %.4f   Δd = %.4f', res.d_high, res.d_low, res.delta_d);
        if isnan(res.tu)
            tuStr = 'n/a';
        else
            tuStr = sprintf('%.4f s', res.tu);
        end
        L{end+1} = sprintf('极限环：幅值 a = %.1f tick/s    Tu = %s', res.amp, tuStr);
        if isempty(res.ku) || isnan(res.tu)
            L{end+1} = '[无效] 幅值或周期无法判定：请加大 dHigh/dLow 差距后重跑';
            txt = strjoin(L, newline); return;
        end
        L{end+1} = sprintf('Ku = 4·Δd/(π·a) = %.8f', res.ku);
        if isfield(res, 'kp')
            L{end+1} = sprintf('ZN 表：建议 kp = 0.45·Ku = %.8f    ki = 0.54·Ku/Tu = %.8f', res.kp, res.ki);
        end

    case 'drop04'
        if ~res.ok
            L{end+1} = sprintf('[无效] %s', res.msg);
            txt = strjoin(L, newline); return;
        end
        L{end+1} = sprintf('稳态均值 = %.1f tick/s', res.mean);
        L{end+1} = sprintf('σ (样本标准差 ÷N-1) = %.2f tick/s', res.sd);
        L{end+1} = sprintf('速度量化台阶 = %.1f tick/s   →  σ = %.2f 个台阶', res.step, res.sigma_steps);
        L{end+1} = sprintf('最大减速度 maxDecel(EMA α=%g) = %.1f tick/s²', res.alpha, res.max_decel);
        L{end+1} = sprintf('Δω_th = k_σ·σ = %g×%.2f = %.2f tick/s  (≈%.2f 个台阶)', ...
                           res.k_sigma, res.sd, res.drop_threshold, res.threshold_steps);
        L{end+1} = sprintf('k_th   = k_rate·maxDecel = %g×%.1f = %.2f tick/s²', ...
                           res.k_rate, res.max_decel, res.drop_rate_threshold);
        if isempty(res.k_boost)
            L{end+1} = 'kBoost：CSV 无 kBoost 且未给 -kp，请加 -kp <当前 kp>';
        else
            L{end+1} = sprintf('kBoost = k_boostRatio·kp = %g×kp = %.8f', res.k_ratio, res.k_boost);
        end
        if ~isempty(res.onboard.sigma)
            L{end+1} = '--- 端上（OpMode）给出的值，用于一致性核对 ---';
            L{end+1} = sprintf('  sigma=%g step=%g maxDecel=%g dropThreshold=%g dropRate=%g kBoost=%g', ...
                               res.onboard.sigma, res.onboard.step, res.onboard.max_decel, ...
                               res.onboard.drop_threshold, res.onboard.drop_rate_threshold, res.onboard.k_boost);
        end
        if res.threshold_steps < 3
            L{end+1} = '  ⚠ 阈值不足 3 个量化台阶：阈值会被量化噪声主导，请提高目标转速或保守取值。';
        end

    case 'ff05'
        if ~res.ok
            L{end+1} = sprintf('[无效] %s', res.msg);
            txt = strjoin(L, newline); return;
        end
        L{end+1} = '测量点 (ω, d_ss)：';
        for k = 1:size(res.points, 1)
            wv = res.points(k, 1); dv = res.points(k, 2);
            L{end+1} = sprintf('  ω = %8.1f tick/s    d_ss = %.4f    Kff = %.8e', wv, dv, dv / wv);
        end
        L{end+1} = sprintf('过原点斜率（推荐填入 kff）= %.8f 占空比/(tick/s)', res.kff_origin);
        if isfield(res, 'slope')
            L{end+1} = sprintf('线性拟合斜率 = %.8f    截距 = %.4f', res.slope, res.intercept);
            L{end+1} = '  截距明显非 0 → 存在静摩擦；当前控制律无 kS 项，可考虑加常数偏置。';
        end

    otherwise
        L{end+1} = '未识别方法：仅输出基本统计';
        if ~isempty(d.t)
            L{end+1} = sprintf('转速 min/mean/max = %.1f / %.1f / %.1f', min(d.vel), mean(d.vel), max(d.vel));
            L{end+1} = sprintf('占空比 min/mean/max = %.4f / %.4f / %.4f', min(d.duty), mean(d.duty), max(d.duty));
        end
end
txt = strjoin(L, newline);
end

function [span, rate] = spanAndRate(t)
% 分段累加真实总时长（step01 的 t 在阶跃处重新从 0 开始）
span = 0; rate = 0;
if numel(t) < 2, return; end
startT = t(1);
for k = 2:numel(t)
    if t(k) < t(k-1)
        span = span + (t(k-1) - startT);
        startT = t(k);
    end
end
span = span + (t(end) - startT);
if span > 0, rate = numel(t) / span; end
end

function writeUtf8(path, txt)
fid = fopen(path, 'w', 'n', 'UTF-8');
if fid < 0, return; end
fwrite(fid, unicode2native(txt, 'UTF-8'), 'uint8');
fclose(fid);
end

% =========================================================================
% 绘图
% =========================================================================

function makePlots(d, res, method, pngPath)
fonts = listfonts; cjk = 'Helvetica';
for nm = {'Microsoft YaHei', 'Microsoft YaHei UI', 'SimHei', 'SimSun'}
    if any(strcmpi(fonts, nm{1})), cjk = nm{1}; break; end
end
set(groot, 'defaultAxesFontName', cjk, 'defaultTextFontName', cjk, ...
    'defaultLegendFontName', cjk, 'defaultAxesFontSize', 10);

fig = figure('Visible', 'off', 'Position', [100 100 1000 700], 'Color', 'w');
ax1 = subplot(2, 1, 1); hold(ax1, 'on'); grid(ax1, 'on');
ax2 = subplot(2, 1, 2); hold(ax2, 'on'); grid(ax2, 'on');

switch method
    case 'step01'
        if ~res.ok
            plot(ax1, d.t, d.vel, 'b-'); xlabel(ax1, '时间 (s)'); ylabel(ax1, '转速 (tick/s)');
        else
            idx = 0;
            for k = 1:numel(d.events)
                if contains(d.events{k}{2}, 'step applied here'), idx = d.events{k}{1}; break; end
            end
            preT = d.t(1:idx); postT = d.t(idx+1:end);
            tShift = preT - preT(end);
            legendStr = {'实测 ω (阶跃前)', '实测 ω (阶跃后)'};
            plot(ax1, tShift, d.vel(1:idx), 'b-', 'LineWidth', 1.2);
            plot(ax1, postT, d.vel(idx+1:end), 'b-', 'LineWidth', 1.3);
            yline(ax1, res.omega0, ':', 'Color', [0.5 0.5 0.5]);
            yline(ax1, res.omega_ss, ':', 'Color', [0.5 0.5 0.5]);
            if ~isempty(res.fit)
                tt = linspace(0, postT(end), 400);
                vv = res.fit.v0 + res.fit.K * (1 - exp(-tt / res.fit.tau));
                plot(ax1, tt, vv, 'r--', 'LineWidth', 1.4);
                legendStr{end+1} = sprintf('最小二乘拟合 τ=%.3fs', res.fit.tau);
            end
            if ~isnan(res.tau_632)
                xline(ax1, res.tau_632, 'g--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
                legendStr{end+1} = sprintf('t63.2%% = %.3fs', res.tau_632);
            end
            xline(ax1, 0, 'k-', 'LineWidth', 0.8, 'HandleVisibility', 'off');
            xlabel(ax1, '时间 (s) （0 = 阶跃时刻）'); ylabel(ax1, '转速 (tick/s)');
            title(ax1, sprintf('6.1 阶跃响应辨识  Kd=%.4f (tick/s)/duty', res.kd));
            legend(ax1, legendStr, 'Location', 'southeast', 'FontSize', 8);
            plot(ax2, tShift, d.duty(1:idx), 'Color', [0.85 0.45 0.15], 'LineWidth', 1.2);
            plot(ax2, postT, d.duty(idx+1:end), 'Color', [0.85 0.45 0.15], 'LineWidth', 1.3);
            xline(ax2, 0, 'k-', 'LineWidth', 0.8, 'HandleVisibility', 'off');
            xlabel(ax2, '时间 (s)'); ylabel(ax2, '占空比'); title(ax2, '占空比（开环阶跃）');
        end
    otherwise
        plot(ax1, d.t, d.vel, 'b-', 'LineWidth', 1.2);
        xlabel(ax1, '时间 (s)'); ylabel(ax1, '转速 (tick/s)');
        title(ax1, sprintf('%s 实测转速', method));
        plot(ax2, d.t, d.duty, 'Color', [0.85 0.45 0.15], 'LineWidth', 1.2);
        xlabel(ax2, '时间 (s)'); ylabel(ax2, '占空比');
        if ismember(method, {'zn02', 'relay03'}) && res.ok
            for a = [ax1 ax2]
                if isfield(res, 'mean') && ~isempty(res.mean)
                    yline(a, res.mean, ':', 'Color', [0.5 0.5 0.5]);
                    yline(a, res.mean + res.amp, 'r:', 'HandleVisibility', 'off');
                    yline(a, res.mean - res.amp, 'r:', 'HandleVisibility', 'off');
                end
            end
            if isnan(res.tu), tuStr = 'n/a'; else, tuStr = sprintf('%.4fs', res.tu); end
            title(ax1, sprintf('%s  幅值 a=%.1f  Tu=%s', method, res.amp, tuStr));
        end
        if strcmp(method, 'ff05') && res.ok
            cla(ax2);
            w = res.points(:, 1); ds = res.points(:, 2);
            plot(ax2, w, ds, 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'k');
            ww = linspace(0, max(w) * 1.05, 100);
            plot(ax2, ww, res.kff_origin * ww, 'r-', 'LineWidth', 1.4);
            if isfield(res, 'slope')
                plot(ax2, ww, res.slope * ww + res.intercept, 'b--', 'LineWidth', 1.2);
                legend(ax2, {'测量点', sprintf('过原点 Kff=%.3e', res.kff_origin), ...
                             sprintf('线性拟合 %.3e·ω+%.4f', res.slope, res.intercept)}, ...
                       'Location', 'northwest', 'FontSize', 8);
            else
                legend(ax2, {'测量点', sprintf('过原点 Kff=%.3e', res.kff_origin)}, ...
                       'Location', 'northwest', 'FontSize', 8);
            end
            xlabel(ax2, '转速 ω (tick/s)'); ylabel(ax2, '稳态占空比 d_{ss}');
            title(ax2, '6.5 前馈增益标定');
        end
end

[~, name, ext] = fileparts(d.path);
sgtitle(fig, [name ext], 'FontSize', 9);
drawnow;
exportgraphics(fig, pngPath, 'Resolution', 150);
close(fig);
end
