function plot_speed(varargin)
%PLOT_SPEED 飞轮「转速图」生成 + 统计（读处理完的数据表，或直接读原始 Dashboard 遥测 CSV）
%
% 用法（在 MATLAB 里 cd 到本文件目录后）：
%   plot_speed                                  % 自动取当前目录下第一个 *_clean.csv
%   plot_speed('log4_clean.csv')                % 指定处理完的数据表
%   plot_speed('PrototypeShooterTest xxx.csv')  % 也支持原始 Dashboard 导出（脚本内部自动处理）
%   plot_speed({'a_clean.csv','b_clean.csv'})   % 多份叠在一起比较转速
%   plot_speed('log4_clean.csv','-out','figs','-title','起飞轮 2250')
%
% 选项：
%   -out DIR      图片与统计报告输出目录（默认与数据文件同目录）
%   -tail S       稳态统计窗口长度，秒（默认 2）
%   -title STR    图标题
%   -noplot       只打印统计、不出图
%
% 输出：
%   <名字>_speed.png   转速专图（目标 vs 实测 + 量化台阶网格 + 掉速事件着色 + 误差子图）
%   <名字>_diag.png    四联诊断图（转速 / 误差 / 占空比与 FF·P·I 分量 / 电流 + boost）
%   <名字>_stats.txt   控制台打印的同一份统计
%
% 数据表格式（export_clean_table.py 生成）：以 '#' 开头的元数据行 + 表头 + 数据行，
%   列为 t,target,velocity,error,duty,ff,i,p,boost,boost_term,current
%
% 依赖：仅 base MATLAB（不用任何 toolbox）。中文标签按《交接说明》§3.2 的方式设置字体。

clc;
cfg = struct('out', '', 'tail', 2.0, 'title', '', 'plot', true);
inputs = {};
i = 1;
while i <= numel(varargin)
    a = varargin{i};
    if ischar(a) && ~isempty(a) && a(1) == '-'
        switch lower(a)
            case '-out',   cfg.out = varargin{i+1}; i = i + 2;
            case '-tail',  cfg.tail = varargin{i+1}; i = i + 2;
            case '-title', cfg.title = varargin{i+1}; i = i + 2;
            case '-noplot', cfg.plot = false; i = i + 1;
            otherwise, error('未知选项：%s', a);
        end
    else
        inputs{end+1} = a; %#ok<AGROW>
        i = i + 1;
    end
end
if isempty(inputs)
    f = dir('*_clean.csv');
    if isempty(f)
        error('当前目录没有 *_clean.csv，请给出文件路径');
    end
    inputs = {f(1).name};
end
files = {};
for k = 1:numel(inputs)
    p = inputs{k};
    if isfolder(p)
        f = dir(fullfile(p, '*_clean.csv'));
        for j = 1:numel(f), files{end+1} = fullfile(f(j).folder, f(j).name); end %#ok<AGROW>
    else
        files{end+1} = p; %#ok<AGROW>
    end
end

ds = {};
for k = 1:numel(files)
    if ~isfile(files{k})
        fprintf('[跳过] 不存在：%s\n', files{k});
        continue;
    end
    ds{end+1} = loadOne(files{k}); %#ok<AGROW>
end
if isempty(ds), error('没有可用的数据文件'); end

% ================= 逐个文件统计 =================
report = {};
for k = 1:numel(ds)
    txt = summarize(ds{k}, cfg.tail);
    fprintf('%s\n', txt);
    report{end+1} = txt; %#ok<AGROW>
end

outDir = cfg.out;
if isempty(outDir), outDir = fileparts(ds{1}.path); end
if ~isfolder(outDir), mkdir(outDir); end
fid = fopen(fullfile(outDir, 'plot_speed_stats.txt'), 'w', 'n', 'UTF-8');
if fid > 0
    fwrite(fid, unicode2native(strjoin(report, newline), 'UTF-8'), 'uint8');
    fclose(fid);
end

if cfg.plot
    fonts = listfonts; cjk = 'Helvetica';
    for nm = {'Microsoft YaHei', 'Microsoft YaHei UI', 'SimHei', 'SimSun'}
        if any(strcmpi(fonts, nm{1})), cjk = nm{1}; break; end
    end
    set(groot, 'defaultAxesFontName', cjk, 'defaultTextFontName', cjk, ...
        'defaultLegendFontName', cjk, 'defaultAxesFontSize', 10);
    drawSpeed(ds, outDir, cfg.title);
    drawDiag(ds, outDir, cfg.title);
end
end

% =========================================================================
% 读文件：自动识别「处理完的数据表」与「原始 Dashboard 遥测」
% =========================================================================
function d = loadOne(path)
fid = fopen(path, 'r', 'n', 'UTF-8');
if fid < 0, error('无法打开 %s', path); end
hdr = '';
rows = [];
isRaw = false;
names = {};
rawMap = [];
while true
    line = fgetl(fid);
    if ~ischar(line), break; end
    s = strtrim(line);
    if isempty(s) || s(1) == '#', continue; end
    if isempty(hdr)
        hdr = s;
        names = strtrim(strsplit(s, ','));
        isRaw = contains(hdr, 'Shooter') || startsWith(lower(hdr), 'time,');
        if isRaw, rawMap = rawColumnMap(names); end
        continue;
    end
    parts = strsplit(s, ',');
    if isRaw
        % 原始日志：time 列是 hh:mm:ss.mmm；其余列一律按「列名」取，兼容
        %   详细遥测（FF/I/P/boost/... 10 列）与新的精简 5 行遥测（只有 current/duty/target/velocity/voltage）
        tt = sscanf(parts{1}, '%d:%d:%f');
        if numel(tt) < 3, continue; end
        vals = nan(1, 11);
        vals(1) = tt(1) * 3600 + tt(2) * 60 + tt(3);
        for j = 1:numel(rawMap)
            c = rawMap(j);
            if c > 0 && c <= numel(parts)
                vals(j + 1) = str2double(parts{c});
            end
        end
    else
        % 处理完的数据表：时间/目标/实测/占空比必须有；FF/I/P/boost 等可选列缺失时留 NaN
        if numel(parts) < 5, continue; end
        n = min(numel(parts), 11);
        vals = nan(1, 11);
        vals(1:n) = str2double(parts(1:n));
    end
    if any(isnan(vals([1 2 3 5]))), continue; end
    rows(end+1, :) = vals; %#ok<AGROW>
end
fclose(fid);
if isempty(rows), error('没读到数据行：%s', path); end

d = struct('path', path);
[~, d.name] = fileparts(path);
if isRaw
    % 时间列正常：重建均匀时间轴（毫秒字段不可靠，按行序走，与 export_clean_table.py 同口径）
    t0 = rows(1, 1); span = rows(end, 1) - t0;
    if span <= 0, span = size(rows, 1) * 0.01; end
    d.dt = span / (size(rows, 1) - 1);
    d.t = rows(:, 1) - t0;
    d.raw = true;
else
    d.t = rows(:, 1); d.dt = median(diff(d.t)); d.raw = false;
end
% 两种来源统一为同一列布局：t, target, velocity, error, duty, ff, i, p, boost, bt, current
d.target = rows(:, 2); d.velocity = rows(:, 3);
d.duty = rows(:, 5); d.ff = rows(:, 6); d.i = rows(:, 7); d.p = rows(:, 8);
d.boost = rows(:, 9); d.bt = rows(:, 10); d.current = rows(:, 11);
d.error = d.target - d.velocity;
d.span = d.t(end) - d.t(1);
% 量化台阶 = 速度的最大公约数
g = 0;
for k = 1:numel(d.velocity)
    iv = round(abs(d.velocity(k)));
    if iv > 0, g = gcd(g, iv); end
end
d.step = double(g);
end

% =========================================================================
% 统计
% =========================================================================
function txt = summarize(d, tailSec)
L = {};
L{end+1} = repmat('=', 1, 88);
L{end+1} = sprintf('文件：%s', d.path);
L{end+1} = sprintf('样本 %d   时长 %.2f s   采样间隔 %.1f ms (%.1f Hz)   量化台阶 %.0f tick/s   速度范围 [%.0f, %.0f]', ...
    numel(d.t), d.span, d.dt * 1000, 1 / d.dt, d.step, min(d.velocity), max(d.velocity));
L{end+1} = repmat('=', 1, 88);

ch = find(diff(d.target) ~= 0) + 1;
bnd = [1; ch(:); numel(d.t) + 1];
L{end+1} = sprintf('%-7s %-8s %-9s %-8s %-8s %-9s %-8s %-8s %-8s', ...
    '目标', '时长s', 'v均值', 'v_sd', '台阶数', '误差均值', '误差%', 'duty均值', '饱和%');
seg = struct('target', {}, 'i0', {}, 'i1', {}, 'vmean', {}, 'vsd', {}, 'emean', {}, 'sat', {}, 'dmean', {});
for k = 1:numel(bnd)-1
    i0 = bnd(k); i1 = bnd(k+1) - 1;
    dur = (i1 - i0) * d.dt;
    if dur < 0.3, continue; end
    % 稳态窗口：末尾 tailSec，但不超过段长一半
    win = min(tailSec, 0.5 * dur);
    j0 = max(i0, i1 - round(win / d.dt));
    v = d.velocity(j0:i1); du = d.duty(j0:i1); e = d.error(j0:i1);
    tv = d.target(i0);
    vs = std(v);
    sat = 100 * mean(du >= 0.999);
    if abs(tv) > 1
        epct = 100 * abs(mean(e)) / abs(tv);   % 目标为 0 的停机段不报百分比（会除零）
    else
        epct = NaN;
    end
    L{end+1} = sprintf('%-7.0f %-8.2f %-9.0f %-8.1f %-8.2f %-9.1f %-8.2f %-8.3f %-8.1f', ...
        tv, dur, mean(v), vs, vs / max(d.step, eps), mean(e), epct, mean(du), sat);
    seg(end+1) = struct('target', tv, 'i0', i0, 'i1', i1, 'vmean', mean(v), 'vsd', vs, ...
        'emean', mean(e), 'sat', sat, 'dmean', mean(du)); %#ok<AGROW>
end

% ---- 掉速/发射事件 ----
ev = [];
for k = 1:numel(seg)
    s = seg(k);
    if abs(s.target) < 500 || (s.i1 - s.i0) * d.dt < 2, continue; end
    band = max(0.02 * abs(s.target), d.step);
    delta = max(100, 0.03 * abs(s.target));
    % 跳过起始加速段：从第一次 |e| ≤ 2*band 连续 10 帧之后才开始找事件
    %（否则会把"从上一档加速上来"的过程误判成掉速事件）
    run = 0; j = s.i0;
    while j < s.i1
        if abs(d.target(j) - d.velocity(j)) <= 2 * band
            run = run + 1;
            if run >= 10, break; end
        else
            run = 0;
        end
        j = j + 1;
    end
    while j < s.i1
        if s.target - d.velocity(j) > delta
            k2 = j; vmin = d.velocity(j);
            while k2 < s.i1 && s.target - d.velocity(k2) > band
                vmin = min(vmin, d.velocity(k2)); k2 = k2 + 1;
            end
            run = 0; e2 = k2;
            for q = j+1:min(s.i1, j + 4000)
                if abs(d.target(q) - d.velocity(q)) <= band
                    run = run + 1;
                    if run >= 10, e2 = q - 9; break; end
                else
                    run = 0;
                end
            end
            seg2 = j:max(e2, j);
            ov = 0;
            q2 = min(numel(d.t), e2 + round(2 / d.dt));
            if q2 > e2, ov = max(d.velocity(e2:q2)) - s.target; end
            ev(end+1, :) = [s.target, d.t(j), s.target - vmin, d.t(e2) - d.t(j), ...
                max(d.duty(seg2)), mean(d.boost(seg2) > 0.5), max(d.bt(seg2)), ov]; %#ok<AGROW>
            j = max(e2, j + 1);
        else
            j = j + 1;
        end
    end
end
if ~isempty(ev)
    L{end+1} = '';
    L{end+1} = sprintf('掉速/发射事件 %d 次：', size(ev, 1));
    L{end+1} = sprintf('%-7s %-8s %-8s %-8s %-8s %-9s %-9s %-8s', ...
        '目标', '起始s', '跌幅', '跌%', '恢复s', 'duty峰', 'boost占比', '过冲');
    for k = 1:size(ev, 1)
        L{end+1} = sprintf('%-7.0f %-8.2f %-8.0f %-8.1f %-8.3f %-9.3f %-9.0f %-8.0f', ...
            ev(k,1), ev(k,2), ev(k,3), 100*ev(k,3)/max(ev(k,1),eps), ev(k,4), ev(k,5), 100*ev(k,6), ev(k,8)); %#ok<AGROW>
    end
    L{end+1} = sprintf('  跌幅中位 %.0f tick/s (%.1f%%)   恢复中位 %.3f s (最长 %.3f)   过冲中位 %.0f   归零点：%d', ...
        median(ev(:,3)), 100*median(ev(:,3))/median(ev(:,1)), median(ev(:,4)), max(ev(:,4)), median(ev(:,8)), sum(ev(:,5)>=0.999));
end
L{end+1} = sprintf('占空比：均值 %.3f   饱和(>=0.999) 占比 %.1f%%   boost 触发占比 %.1f%%', ...
    mean(d.duty), 100*mean(d.duty >= 0.999), 100*mean(d.boost > 0.5));
txt = strjoin(L, newline);
end

% =========================================================================
% 图 1：转速专图
% =========================================================================
function drawSpeed(ds, outDir, ttl)
fig = figure('Visible', 'off', 'Position', [80 80 1200 760], 'Color', 'w');
ax1 = subplot(2, 1, 1); hold(ax1, 'on'); grid(ax1, 'on');
ax2 = subplot(2, 1, 2); hold(ax2, 'on'); grid(ax2, 'on');
cl = lines(max(numel(ds), 1));
for k = 1:numel(ds)
    d = ds{k};
    % ① 量化台阶网格（每档目标 ±1/2/3 个台阶，先画以便压在曲线下面）
    if d.step > 0
        b2 = [1; find(diff(d.target) ~= 0) + 1; numel(d.t) + 1];
        for q = 1:numel(b2) - 1
            tv = d.target(b2(q));
            if tv == 0, continue; end
            xr = [d.t(b2(q)) d.t(b2(q+1) - 1)];
            for kk = 1:3
                plot(ax1, xr, [tv + kk * d.step, tv + kk * d.step], ':', ...
                    'Color', [0.78 0.78 0.78], 'HandleVisibility', 'off');
                plot(ax1, xr, [tv - kk * d.step, tv - kk * d.step], ':', ...
                    'Color', [0.78 0.78 0.78], 'HandleVisibility', 'off');
            end
        end
    end
    % ② 曲线
    plot(ax1, d.t, d.velocity, '-', 'Color', cl(k,:), 'LineWidth', 0.9);
    plot(ax1, d.t, d.target, '--', 'Color', [0.85 0.2 0.2], 'LineWidth', 1.0);
    plot(ax2, d.t, d.error, '-', 'Color', cl(k,:), 'LineWidth', 0.8);
    % 目标变化竖线
    ch = find(diff(d.target) ~= 0) + 1;
    for j = 1:numel(ch)
        xline(ax1, d.t(ch(j)), ':', 'Color', [0.6 0.6 0.6], 'HandleVisibility', 'off');
        xline(ax2, d.t(ch(j)), ':', 'Color', [0.6 0.6 0.6], 'HandleVisibility', 'off');
    end
    % ③ 明显掉速/发射区间着色（误差 > max(100, 3%目标)）
    in = d.error > max(100, 0.03 * abs(d.target));
    y1 = ylim(ax1); y2 = ylim(ax2);
    j = 1;
    while j <= numel(in)
        if in(j)
            k2 = j;
            while k2 < numel(in) && in(k2 + 1), k2 = k2 + 1; end
            patch(ax1, [d.t(j) d.t(k2) d.t(k2) d.t(j)], [y1(1) y1(1) y1(2) y1(2)], ...
                [1 0.7 0.3], 'FaceAlpha', 0.28, 'EdgeColor', 'none', 'HandleVisibility', 'off');
            patch(ax2, [d.t(j) d.t(k2) d.t(k2) d.t(j)], [y2(1) y2(1) y2(2) y2(2)], ...
                [1 0.7 0.3], 'FaceAlpha', 0.28, 'EdgeColor', 'none', 'HandleVisibility', 'off');
            j = k2 + 1;
        else
            j = j + 1;
        end
    end
    ylim(ax1, y1); ylim(ax2, y2);
end
d1 = ds{1};
yline(ax2, 0, 'k-', 'HandleVisibility', 'off');
% 误差带：±2% 与 ±1 个量化台阶（取最大目标做参考）
tmax = max(arrayfun(@(q) max(abs(ds{q}.target)), 1:numel(ds)));
yline(ax2, 0.02 * tmax, 'r:', 'LineWidth', 1, 'HandleVisibility', 'off');
yline(ax2, -0.02 * tmax, 'r:', 'LineWidth', 1, 'HandleVisibility', 'off');
if d1.step > 0
    yline(ax2, d1.step, 'g:', 'LineWidth', 1, 'HandleVisibility', 'off');
    yline(ax2, -d1.step, 'g:', 'LineWidth', 1, 'HandleVisibility', 'off');
end
xlabel(ax2, '时间 (s)');
ylabel(ax1, '转速 (tick/s)');
ylabel(ax2, '误差 = 目标 − 实测 (tick/s)');
title(ax1, '转速：目标(红虚线) vs 实测(实线)；灰虚线=目标变化点');
title(ax2, sprintf('误差；红点线=±2%%目标(±%.0f)，绿点线=±1 个量化台阶(±%.0f)', 0.02*tmax, d1.step));
if isempty(ttl), ttl = ds{1}.name; end
sgtitle(fig, ttl, 'FontSize', 10);
legend(ax1, {'实测 ω', '目标 ω*'}, 'Location', 'southeast');
drawnow;
exportgraphics(fig, fullfile(outDir, [ds{1}.name '_speed.png']), 'Resolution', 150);
close(fig);
fprintf('[图] %s\n', fullfile(outDir, [ds{1}.name '_speed.png']));
end

% =========================================================================
% 图 2：四联诊断图
% =========================================================================
function drawDiag(ds, outDir, ttl)
d = ds{1};
fig = figure('Visible', 'off', 'Position', [80 80 1200 900], 'Color', 'w');
ax = gobjects(4, 1);
for k = 1:4, ax(k) = subplot(4, 1, k); hold(ax(k), 'on'); grid(ax(k), 'on'); end

plot(ax(1), d.t, d.velocity, 'b-', 'LineWidth', 0.9);
plot(ax(1), d.t, d.target, 'r--', 'LineWidth', 1.0);
ylabel(ax(1), '转速 (tick/s)');
title(ax(1), '① 转速：目标 vs 实测');

plot(ax(2), d.t, d.error, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 0.8);
yline(ax(2), 0, 'k-', 'HandleVisibility', 'off');
if d.step > 0
    yline(ax(2), d.step, 'r:', 'HandleVisibility', 'off');
    yline(ax(2), -d.step, 'r:', 'HandleVisibility', 'off');
end
ylabel(ax(2), '误差 (tick/s)');
title(ax(2), sprintf('② 误差（红点线=±1 个量化台阶 %.0f tick/s）', d.step));

plot(ax(3), d.t, d.duty, '-', 'Color', [0.85 0.45 0.15], 'LineWidth', 1.0);
plot(ax(3), d.t, d.ff, '-', 'Color', [0.12 0.47 0.71], 'LineWidth', 0.8);
plot(ax(3), d.t, d.i, '-', 'Color', [0.17 0.63 0.17], 'LineWidth', 0.8);
plot(ax(3), d.t, d.p, '-', 'Color', [0.58 0.40 0.74], 'LineWidth', 0.8);
plot(ax(3), d.t, d.ff + d.i + d.p, 'k:', 'LineWidth', 0.7);
yline(ax(3), 1, 'r-', 'HandleVisibility', 'off');
yline(ax(3), 0, 'r-', 'HandleVisibility', 'off');
ylim(ax(3), [-0.05 1.05]);
ylabel(ax(3), '占空比');
legend(ax(3), {'duty', 'FF', 'I', 'P', 'FF+P+I'}, 'Location', 'eastoutside');
title(ax(3), '③ 占空比与分量（黑点线超过上限=被限幅截掉）');

plot(ax(4), d.t, d.current, '-', 'Color', [0.55 0.34 0.29], 'LineWidth', 0.8);
if any(d.boost > 0.5)
    on = d.boost > 0.5;
    yy = ylim(ax(4));
    j = 1;
    while j <= numel(on)
        if on(j)
            k2 = j;
            while k2 < numel(on) && on(k2 + 1), k2 = k2 + 1; end
            patch(ax(4), [d.t(j) d.t(k2) d.t(k2) d.t(j)], [yy(1) yy(1) yy(2) yy(2)], ...
                [0.4 0.8 0.4], 'FaceAlpha', 0.25, 'EdgeColor', 'none', 'HandleVisibility', 'off');
            j = k2 + 1;
        else
            j = j + 1;
        end
    end
    ylim(ax(4), yy);
end
ylabel(ax(4), '电流 (A)');
xlabel(ax(4), '时间 (s)');
title(ax(4), '④ 电机电流（绿色阴影=掉速补偿 boost 生效）');

if isempty(ttl), ttl = d.name; end
sgtitle(fig, ttl, 'FontSize', 10);
drawnow;
exportgraphics(fig, fullfile(outDir, [d.name '_diag.png']), 'Resolution', 150);
close(fig);
fprintf('[图] %s\n', fullfile(outDir, [d.name '_diag.png']));
end

% =========================================================================
% 原始 Dashboard 日志的列名 → 统一列布局的映射
% 统一布局（11 列）：t, target, velocity, error, duty, ff, i, p, boost, bt, current
% 返回 10 个「原始列号」（落在 part 的第几列）；找不到记 0 → 该列留 NaN
% =========================================================================
function map = rawColumnMap(names)
cands = { ...
    {'Shooter target (tick/s)', 'target (tick/s)'}, ...        % target
    {'Shooter velocity (tick/s)', 'velocity (tick/s)'}, ...     % velocity
    {'Shooter error (tick/s)', 'error (tick/s)'}, ...           % error（可缺，后面用 target-velocity 重算）
    {'Shooter power (duty)', 'Shooter duty', 'duty', 'power'}, ... % duty
    {'Shooter FF term', 'Shooter FF', 'FF term'}, ...           % ff
    {'Shooter I term', 'Shooter I', 'I term'}, ...              % i
    {'Shooter P term', 'Shooter P', 'P term'}, ...              % p
    {'Shooter boost', 'boost'}, ...                             % boost 标志
    {'Shooter boost term', 'boost term'}, ...                   % boost 项
    {'Shooter current (A)', 'current (A)'}};                    % current
map = zeros(1, numel(cands));
for j = 1:numel(cands)
    for c = 1:numel(names)
        hit = false;
        for k = 1:numel(cands{j})
            if strcmp(names{c}, cands{j}{k}), map(j) = c; hit = true; break; end
        end
        if hit, break; end
    end
    if map(j) == 0   % 严格匹配不到时退化为「包含」
        for c = 1:numel(names)
            hit = false;
            for k = 1:numel(cands{j})
                if contains(names{c}, cands{j}{k}), map(j) = c; hit = true; break; end
            end
            if hit, break; end
        end
    end
end
end
