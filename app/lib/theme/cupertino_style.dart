import 'dart:async';
import 'dart:ui' show ImageFilter;

import 'package:flutter/cupertino.dart'
    show CupertinoActivityIndicator, CupertinoIcons;
import 'package:flutter/material.dart';
import 'package:flutter/physics.dart';
import 'package:flutter/services.dart';

import '../core/fsm_state.dart';

export 'package:flutter/cupertino.dart'
    show CupertinoActivityIndicator, CupertinoIcons, CupertinoScrollBehavior;

// ============================================================================
// Apple Cupertino 스타일 디자인 시스템 (Achromatic / High-Contrast Monochrome)
//
// - 색상  : 옅은 회색 배경 + 검정/짙은 회색 포인트
// - 형태  : Squircle(연속 곡률) 카드/버튼 + 은은한 Soft Drop Shadow
// - 효과  : 지도 위 오버레이 패널에 Glassmorphism(Frosted glass)
// - 모션  : Spring Animation + Fluid Micro-interaction
//
// 이 파일은 순수 UI 스타일 계층이며 통신/상태 로직과는 무관하다.
// ============================================================================

/// 모노크롬 팔레트.
abstract final class AppColors {
  static const background = Color(0xFFF2F2F2);
  static const surface = Color(0xFFFFFFFF);
  static const ink = Color(0xFF111111);
  static const inkSecondary = Color(0xFF48484A);
  static const inkTertiary = Color(0xFF8E8E93);
  static const disabledFill = Color(0xFFD9D9DB);

  /// 검정 6% / 12% 오버레이 (미묘한 채움 / 헤어라인).
  static const fillSubtle = Color(0x0F000000);
  static const hairline = Color(0x1F000000);
}

/// 은은한 Soft Drop Shadow.
abstract final class AppShadows {
  static const soft = [
    BoxShadow(
      color: Color(0x14000000),
      blurRadius: 24,
      spreadRadius: -2,
      offset: Offset(0, 10),
    ),
    BoxShadow(
      color: Color(0x0A000000),
      blurRadius: 3,
      offset: Offset(0, 1),
    ),
  ];

  static const floating = [
    BoxShadow(
      color: Color(0x1A000000),
      blurRadius: 32,
      spreadRadius: -6,
      offset: Offset(0, 12),
    ),
  ];
}

/// 타이포그래피 (iOS Human Interface Guidelines 느낌의 굵기/자간).
abstract final class AppText {
  static const largeTitle = TextStyle(
    fontSize: 28,
    fontWeight: FontWeight.w700,
    letterSpacing: -0.6,
    height: 1.15,
    color: AppColors.ink,
  );
  static const title = TextStyle(
    fontSize: 20,
    fontWeight: FontWeight.w600,
    letterSpacing: -0.3,
    color: AppColors.ink,
  );
  static const headline = TextStyle(
    fontSize: 16,
    fontWeight: FontWeight.w600,
    letterSpacing: -0.2,
    color: AppColors.ink,
  );
  static const body = TextStyle(
    fontSize: 15,
    fontWeight: FontWeight.w400,
    height: 1.4,
    letterSpacing: -0.1,
    color: AppColors.ink,
  );
  static const callout = TextStyle(
    fontSize: 14,
    fontWeight: FontWeight.w400,
    height: 1.4,
    color: AppColors.inkSecondary,
  );
  static const caption = TextStyle(
    fontSize: 12,
    fontWeight: FontWeight.w500,
    color: AppColors.inkTertiary,
  );
}

/// Squircle(연속 곡률 둥근 사각형) 모양.
RoundedSuperellipseBorder squircle(
  double radius, {
  BorderSide side = BorderSide.none,
}) {
  return RoundedSuperellipseBorder(
    borderRadius: BorderRadius.circular(radius),
    side: side,
  );
}

// ---------------------------------------------------------------------------
// Spring
// ---------------------------------------------------------------------------

/// 스프링 시뮬레이션을 [Curve]로 사용한다. 부드럽게 도달하며 약간의 오버슈트를 준다.
class SpringCurve extends Curve {
  SpringCurve({
    double stiffness = 200,
    double dampingRatio = 0.82,
    double mass = 1,
  }) : _simulation = SpringSimulation(
          SpringDescription.withDampingRatio(
            mass: mass,
            stiffness: stiffness,
            ratio: dampingRatio,
          ),
          0,
          1,
          0,
        ) {
    const step = 1 / 120;
    var last = 0.0;
    for (var t = 0.0; t < 4.0; t += step) {
      if ((_simulation.x(t) - 1).abs() > 0.002) last = t;
    }
    settleSeconds = last + step;
  }

  final SpringSimulation _simulation;

  /// 목표값에 안착하기까지 걸리는 시간(초).
  late final double settleSeconds;

  Duration get duration =>
      Duration(milliseconds: (settleSeconds * 1000).round());

  @override
  double transformInternal(double t) => _simulation.x(t * settleSeconds);
}

abstract final class AppSprings {
  /// 화면 전환 / 시트 / 패널 등장 (살짝 출렁이며 안착).
  static final smooth = SpringCurve(stiffness: 200, dampingRatio: 0.82);

  /// 작은 요소의 등장 (쫀득한 탄성).
  static final bouncy = SpringCurve(stiffness: 260, dampingRatio: 0.62);

  /// 버튼 눌림/복귀 마이크로 인터랙션.
  static final press = SpringDescription.withDampingRatio(
    mass: 1,
    stiffness: 650,
    ratio: 0.55,
  );

  /// 지도 위 로봇 이동 등 연속적으로 목표가 바뀌는 값을 따라가는 스프링.
  static final follow = SpringDescription.withDampingRatio(
    mass: 1,
    stiffness: 260,
    ratio: 0.9,
  );

  static const decorationDuration = Duration(milliseconds: 280);
  static const decorationCurve = Curves.easeOutCubic;
}

/// 누르면 쫀득하게 줄었다가 스프링으로 복귀하는 마이크로 인터랙션 래퍼.
class SpringPressable extends StatefulWidget {
  const SpringPressable({
    super.key,
    required this.child,
    this.onTap,
    this.pressedScale = 0.95,
    this.haptics = true,
  });

  final Widget child;
  final VoidCallback? onTap;
  final double pressedScale;
  final bool haptics;

  @override
  State<SpringPressable> createState() => _SpringPressableState();
}

class _SpringPressableState extends State<SpringPressable>
    with SingleTickerProviderStateMixin {
  late final AnimationController _controller =
      AnimationController.unbounded(vsync: this, value: 1);

  bool get _enabled => widget.onTap != null;

  void _animateTo(double target) {
    _controller.animateWith(
      SpringSimulation(
        AppSprings.press,
        _controller.value,
        target,
        _controller.velocity,
      ),
    );
  }

  @override
  void didUpdateWidget(covariant SpringPressable oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (!_enabled && oldWidget.onTap != null) _animateTo(1);
  }

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return GestureDetector(
      behavior: HitTestBehavior.opaque,
      onTapDown: _enabled ? (_) => _animateTo(widget.pressedScale) : null,
      onTapUp: _enabled ? (_) => _animateTo(1) : null,
      onTapCancel: _enabled ? () => _animateTo(1) : null,
      onTap: _enabled
          ? () {
              if (widget.haptics) HapticFeedback.selectionClick();
              widget.onTap!();
            }
          : null,
      child: AnimatedBuilder(
        animation: _controller,
        builder: (context, child) =>
            Transform.scale(scale: _controller.value, child: child),
        child: widget.child,
      ),
    );
  }
}

/// 위젯이 처음 나타날 때 스프링으로 떠오르게 한다.
///
/// BackdropFilter(Glassmorphism)는 Opacity 안에서 블러가 깨지므로, 글래스 패널을
/// 감쌀 때는 [fade]를 끈다.
class SpringEntrance extends StatefulWidget {
  const SpringEntrance({
    super.key,
    required this.child,
    this.delay = Duration.zero,
    this.offset = const Offset(0, 24),
    this.fade = true,
  });

  final Widget child;
  final Duration delay;
  final Offset offset;
  final bool fade;

  @override
  State<SpringEntrance> createState() => _SpringEntranceState();
}

class _SpringEntranceState extends State<SpringEntrance>
    with SingleTickerProviderStateMixin {
  late final AnimationController _controller =
      AnimationController(vsync: this, duration: AppSprings.bouncy.duration);
  late final Animation<double> _curved =
      CurvedAnimation(parent: _controller, curve: AppSprings.bouncy);
  Timer? _timer;

  @override
  void initState() {
    super.initState();
    if (widget.delay == Duration.zero) {
      _controller.forward();
    } else {
      _timer = Timer(widget.delay, () {
        if (mounted) _controller.forward();
      });
    }
  }

  @override
  void dispose() {
    _timer?.cancel();
    _controller.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return AnimatedBuilder(
      animation: _curved,
      child: widget.child,
      builder: (context, child) {
        final v = _curved.value;
        Widget result = Transform.translate(
          offset: widget.offset * (1 - v),
          child: Transform.scale(scale: 0.94 + 0.06 * v, child: child),
        );
        if (widget.fade) {
          result = Opacity(opacity: v.clamp(0.0, 1.0).toDouble(), child: result);
        }
        return result;
      },
    );
  }
}

/// [AnimatedSwitcher]용 스프링 전환 (스케일 + 페이드).
Widget springSwitcherTransition(Widget child, Animation<double> animation) {
  final scale = CurvedAnimation(
    parent: animation,
    curve: AppSprings.bouncy,
    reverseCurve: Curves.easeInCubic,
  );
  return FadeTransition(
    opacity: CurvedAnimation(parent: animation, curve: Curves.easeOut),
    child: ScaleTransition(
      scale: Tween<double>(begin: 0.8, end: 1).animate(scale),
      child: child,
    ),
  );
}

/// 화면 전환: 오른쪽에서 스프링으로 밀려 들어오고, 아래 화면은 살짝 패럴랙스.
class SpringPageRoute<T> extends PageRouteBuilder<T> {
  SpringPageRoute({required WidgetBuilder builder})
      : super(
          transitionDuration: AppSprings.smooth.duration,
          reverseTransitionDuration: const Duration(milliseconds: 320),
          pageBuilder: (context, animation, secondaryAnimation) =>
              builder(context),
          transitionsBuilder: (context, animation, secondaryAnimation, child) {
            final primary = CurvedAnimation(
              parent: animation,
              curve: AppSprings.smooth,
              reverseCurve: Curves.easeInCubic,
            );
            final secondary = CurvedAnimation(
              parent: secondaryAnimation,
              curve: Curves.easeOutCubic,
              reverseCurve: Curves.easeInCubic,
            );
            return SlideTransition(
              position: Tween<Offset>(
                begin: Offset.zero,
                end: const Offset(-0.22, 0),
              ).animate(secondary),
              child: SlideTransition(
                position: Tween<Offset>(
                  begin: const Offset(1, 0),
                  end: Offset.zero,
                ).animate(primary),
                child: DecoratedBox(
                  decoration: const BoxDecoration(
                    boxShadow: [
                      BoxShadow(
                        color: Color(0x26000000),
                        blurRadius: 30,
                        offset: Offset(-8, 0),
                      ),
                    ],
                  ),
                  child: child,
                ),
              ),
            );
          },
        );
}

// ---------------------------------------------------------------------------
// Surfaces
// ---------------------------------------------------------------------------

/// Squircle 카드 + Soft Drop Shadow.
class AppCard extends StatelessWidget {
  const AppCard({
    super.key,
    required this.child,
    this.padding = const EdgeInsets.all(16),
    this.radius = 22,
    this.color = AppColors.surface,
    this.onTap,
  });

  final Widget child;
  final EdgeInsetsGeometry padding;
  final double radius;
  final Color color;
  final VoidCallback? onTap;

  @override
  Widget build(BuildContext context) {
    final card = AnimatedContainer(
      duration: AppSprings.decorationDuration,
      curve: AppSprings.decorationCurve,
      padding: padding,
      decoration: ShapeDecoration(
        color: color,
        shape: squircle(radius),
        shadows: AppShadows.soft,
      ),
      child: child,
    );
    if (onTap == null) return card;
    return SpringPressable(onTap: onTap, pressedScale: 0.98, child: card);
  }
}

/// Glassmorphism (Frosted glass) 패널. 지도 위에 떠 있는 알림/컨트롤에 사용한다.
class GlassPanel extends StatelessWidget {
  const GlassPanel({
    super.key,
    required this.child,
    this.padding = EdgeInsets.zero,
    this.radius = 24,
    this.blur = 22,
    this.dark = false,
  });

  final Widget child;
  final EdgeInsetsGeometry padding;
  final double radius;
  final double blur;

  /// true 면 검은 유리(고대비), false 면 밝은 유리.
  final bool dark;

  @override
  Widget build(BuildContext context) {
    final shape = squircle(
      radius,
      side: BorderSide(
        color: dark ? const Color(0x2EFFFFFF) : const Color(0xB3FFFFFF),
      ),
    );
    final colors = dark
        ? const [Color(0xA6141414), Color(0x8C1C1C1E)]
        : const [Color(0xB8FFFFFF), Color(0x8CF7F7F7)];

    return DecoratedBox(
      decoration: ShapeDecoration(
        shape: squircle(radius),
        shadows: AppShadows.floating,
      ),
      child: ClipPath(
        clipper: ShapeBorderClipper(shape: shape),
        clipBehavior: Clip.antiAlias,
        child: BackdropFilter(
          filter: ImageFilter.blur(sigmaX: blur, sigmaY: blur),
          child: DecoratedBox(
            decoration: ShapeDecoration(
              shape: shape,
              gradient: LinearGradient(
                begin: Alignment.topLeft,
                end: Alignment.bottomRight,
                colors: colors,
              ),
            ),
            child: Padding(padding: padding, child: child),
          ),
        ),
      ),
    );
  }
}

/// 전체 폭 유리 바 (AppBar의 flexibleSpace 용).
class GlassBar extends StatelessWidget {
  const GlassBar({super.key});

  @override
  Widget build(BuildContext context) {
    return ClipRect(
      child: BackdropFilter(
        filter: ImageFilter.blur(sigmaX: 24, sigmaY: 24),
        child: const DecoratedBox(
          decoration: BoxDecoration(
            color: Color(0xB8F2F2F2),
            border: Border(bottom: BorderSide(color: Color(0x14000000))),
          ),
          child: SizedBox.expand(),
        ),
      ),
    );
  }
}

// ---------------------------------------------------------------------------
// Controls
// ---------------------------------------------------------------------------

enum AppButtonVariant { filled, outlined, tinted }

/// Squircle 버튼 (스프링 눌림 + 부드러운 활성/비활성 전환).
class AppButton extends StatelessWidget {
  const AppButton({
    super.key,
    required this.label,
    required this.onPressed,
    this.icon,
    this.variant = AppButtonVariant.filled,
    this.compact = false,
    this.loading = false,
  });

  final String label;
  final VoidCallback? onPressed;
  final IconData? icon;
  final AppButtonVariant variant;
  final bool compact;
  final bool loading;

  @override
  Widget build(BuildContext context) {
    final enabled = onPressed != null;
    final radius = compact ? 13.0 : 16.0;

    final Color bg;
    final Color fg;
    BorderSide border = BorderSide.none;
    List<BoxShadow> shadows = const [];
    switch (variant) {
      case AppButtonVariant.filled:
        bg = enabled ? AppColors.ink : AppColors.disabledFill;
        fg = enabled ? Colors.white : AppColors.inkTertiary;
        shadows = enabled ? AppShadows.soft : const [];
      case AppButtonVariant.outlined:
        bg = enabled ? AppColors.surface : const Color(0x99FFFFFF);
        fg = enabled ? AppColors.ink : AppColors.inkTertiary;
        border = const BorderSide(color: AppColors.hairline);
        shadows = enabled ? AppShadows.soft : const [];
      case AppButtonVariant.tinted:
        bg = enabled ? AppColors.fillSubtle : const Color(0x08000000);
        fg = enabled ? AppColors.ink : AppColors.inkTertiary;
    }

    return SpringPressable(
      onTap: enabled && !loading ? onPressed : null,
      child: AnimatedContainer(
        duration: AppSprings.decorationDuration,
        curve: AppSprings.decorationCurve,
        height: compact ? 38 : 50,
        padding: EdgeInsets.symmetric(horizontal: compact ? 14 : 10),
        alignment: Alignment.center,
        decoration: ShapeDecoration(
          color: bg,
          shape: squircle(radius, side: border),
          shadows: shadows,
        ),
        child: TweenAnimationBuilder<Color?>(
          tween: ColorTween(end: fg),
          duration: AppSprings.decorationDuration,
          builder: (context, color, _) {
            final c = color ?? fg;
            return AnimatedSwitcher(
              duration: const Duration(milliseconds: 200),
              child: loading
                  ? CupertinoActivityIndicator(
                      key: const ValueKey('loading'),
                      color: c,
                    )
                  : Row(
                      key: const ValueKey('label'),
                      mainAxisSize: MainAxisSize.min,
                      children: [
                        if (icon != null) ...[
                          Icon(icon, size: compact ? 16 : 18, color: c),
                          const SizedBox(width: 5),
                        ],
                        Flexible(
                          child: FittedBox(
                            fit: BoxFit.scaleDown,
                            child: Text(
                              label,
                              maxLines: 1,
                              style: TextStyle(
                                color: c,
                                fontSize: compact ? 13 : 14,
                                fontWeight: FontWeight.w600,
                                letterSpacing: -0.2,
                              ),
                            ),
                          ),
                        ),
                      ],
                    ),
            );
          },
        ),
      ),
    );
  }
}

/// 정사각 Squircle 아이콘 버튼.
class AppIconButton extends StatelessWidget {
  const AppIconButton({
    super.key,
    required this.icon,
    required this.onTap,
    this.tooltip,
    this.size = 38,
    this.plain = false,
  });

  final IconData icon;
  final VoidCallback? onTap;
  final String? tooltip;
  final double size;

  /// true 면 배경/그림자 없이 아이콘만 (유리 패널 내부용).
  final bool plain;

  @override
  Widget build(BuildContext context) {
    final enabled = onTap != null;
    Widget button = SpringPressable(
      onTap: onTap,
      pressedScale: 0.88,
      child: AnimatedOpacity(
        duration: AppSprings.decorationDuration,
        opacity: enabled ? 1 : 0.35,
        child: Container(
          width: size,
          height: size,
          alignment: Alignment.center,
          decoration: plain
              ? null
              : ShapeDecoration(
                  color: AppColors.surface,
                  shape: squircle(size * 0.34),
                  shadows: AppShadows.soft,
                ),
          child: Icon(icon, size: size * 0.5, color: AppColors.ink),
        ),
      ),
    );
    if (tooltip != null) button = Tooltip(message: tooltip!, child: button);
    return button;
  }
}

/// 작은 Squircle 칩.
class AppChip extends StatelessWidget {
  const AppChip({
    super.key,
    required this.label,
    this.icon,
    this.inverted = false,
  });

  final String label;
  final IconData? icon;
  final bool inverted;

  @override
  Widget build(BuildContext context) {
    final fg = inverted ? Colors.white : AppColors.ink;
    return AnimatedContainer(
      duration: AppSprings.decorationDuration,
      curve: AppSprings.decorationCurve,
      padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 5),
      decoration: ShapeDecoration(
        color: inverted ? AppColors.ink : AppColors.fillSubtle,
        shape: squircle(10),
      ),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          if (icon != null) ...[
            Icon(icon, size: 13, color: fg),
            const SizedBox(width: 4),
          ],
          Text(
            label,
            style: TextStyle(
              color: fg,
              fontSize: 12,
              fontWeight: FontWeight.w600,
              letterSpacing: -0.1,
            ),
          ),
        ],
      ),
    );
  }
}

/// 서브 화면(기록/설정)용 상단 바: 글래스 + 중앙 타이틀 + 뒤로가기.
class AppTopBar extends StatelessWidget implements PreferredSizeWidget {
  const AppTopBar({super.key, required this.title});

  final String title;

  @override
  Size get preferredSize => const Size.fromHeight(kToolbarHeight);

  @override
  Widget build(BuildContext context) {
    final canPop = Navigator.of(context).canPop();
    return AppBar(
      automaticallyImplyLeading: false,
      backgroundColor: Colors.transparent,
      surfaceTintColor: Colors.transparent,
      elevation: 0,
      scrolledUnderElevation: 0,
      centerTitle: true,
      systemOverlayStyle: SystemUiOverlayStyle.dark,
      flexibleSpace: const GlassBar(),
      leadingWidth: 62,
      leading: canPop
          ? Padding(
              padding: const EdgeInsets.only(left: 16),
              child: Center(
                child: AppIconButton(
                  icon: CupertinoIcons.chevron_back,
                  tooltip: '뒤로',
                  onTap: () => Navigator.of(context).maybePop(),
                ),
              ),
            )
          : null,
      title: Text(title, style: AppText.headline),
    );
  }
}

/// 화면 하단에 떠오르는 스프링 스낵바(유리 패널 내용용).
void showAppSnackBar(
  BuildContext context, {
  required Widget child,
  Duration duration = const Duration(seconds: 4),
  double bottomMargin = 12,
}) {
  final messenger = ScaffoldMessenger.of(context);
  messenger.clearSnackBars();
  messenger.showSnackBar(
    SnackBar(
      duration: duration,
      backgroundColor: Colors.transparent,
      elevation: 0,
      padding: EdgeInsets.zero,
      behavior: SnackBarBehavior.floating,
      margin: EdgeInsets.fromLTRB(16, 0, 16, bottomMargin),
      shape: squircle(24),
      clipBehavior: Clip.none, // 유리 패널의 Soft Drop Shadow 가 잘리지 않게.
      content: SpringEntrance(
        fade: false,
        offset: const Offset(0, 36),
        child: child,
      ),
    ),
  );
}

// ---------------------------------------------------------------------------
// FSM 상태의 모노크롬 표현 (fsm_state.dart 는 건드리지 않고 UI 매핑만 추가)
// ---------------------------------------------------------------------------

IconData robotStateIcon(RobotState state) {
  switch (state) {
    case RobotState.idle:
      return CupertinoIcons.pause_circle;
    case RobotState.mapping:
      return CupertinoIcons.map;
    case RobotState.cleaning:
      return CupertinoIcons.sparkles;
    case RobotState.evading:
      return CupertinoIcons.exclamationmark_triangle_fill;
    case RobotState.targetRecleaning:
      return CupertinoIcons.location_fill;
    case RobotState.returning:
      return CupertinoIcons.house;
  }
}

// ---------------------------------------------------------------------------
// Theme
// ---------------------------------------------------------------------------

abstract final class AppTheme {
  static ThemeData light() {
    const scheme = ColorScheme.light(
      primary: AppColors.ink,
      onPrimary: Colors.white,
      secondary: AppColors.inkSecondary,
      onSecondary: Colors.white,
      surface: AppColors.surface,
      onSurface: AppColors.ink,
      error: AppColors.inkSecondary,
      onError: Colors.white,
      surfaceTint: Colors.transparent,
    );
    return ThemeData(
      useMaterial3: true,
      colorScheme: scheme,
      scaffoldBackgroundColor: AppColors.background,
      splashFactory: NoSplash.splashFactory,
      highlightColor: Colors.transparent,
      hoverColor: Colors.transparent,
      dividerColor: AppColors.hairline,
      appBarTheme: const AppBarTheme(
        backgroundColor: Colors.transparent,
        surfaceTintColor: Colors.transparent,
        elevation: 0,
        scrolledUnderElevation: 0,
        foregroundColor: AppColors.ink,
        systemOverlayStyle: SystemUiOverlayStyle.dark,
      ),
      textSelectionTheme: const TextSelectionThemeData(
        cursorColor: AppColors.ink,
        selectionColor: Color(0x26000000),
        selectionHandleColor: AppColors.ink,
      ),
      snackBarTheme: const SnackBarThemeData(
        behavior: SnackBarBehavior.floating,
        backgroundColor: Colors.transparent,
        elevation: 0,
      ),
      progressIndicatorTheme:
          const ProgressIndicatorThemeData(color: AppColors.ink),
      tooltipTheme: TooltipThemeData(
        decoration: ShapeDecoration(
          color: const Color(0xE6111111),
          shape: squircle(10),
        ),
        textStyle: const TextStyle(color: Colors.white, fontSize: 12),
      ),
    );
  }
}
